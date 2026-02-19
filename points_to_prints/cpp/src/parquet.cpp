#include "parquet.hpp"

#include <arrow/array/builder_binary.h>
#include <arrow/scalar.h>
#include <arrow/type_fwd.h>
#include <cstddef>
#include <cstdint>
#include <format>
#include <iostream>
#include <memory>
#include <ostream>
#include <string>
#include <utility>

#include "arrow/api.h"
#include "arrow/io/file.h"
#include "arrow/status.h"
#include "parquet/arrow/reader.h"
#include <parquet/arrow/writer.h>

#include <ogr_geometry.h>
#include <ogr_spatialref.h>

#include "json/json.hpp"

#include "pbar.hpp"

using json = nlohmann::json;

OGRGeometryPtr parseWKBToGeometry(std::shared_ptr<arrow::Scalar> scalar_ptr) {
    if (!scalar_ptr || !arrow::is_binary_like(scalar_ptr->type->id())) {
        std::cerr << "Invalid scalar or not binary type" << std::endl;
        return nullptr;
    }

    // Extract binary data from Arrow BinaryScalar
    auto binary_scalar =
        std::static_pointer_cast<arrow::BinaryScalar>(scalar_ptr);
    const unsigned char *wkb_data =
        reinterpret_cast<const unsigned char *>(binary_scalar->value->data());
    size_t wkb_size = binary_scalar->value->size();

    // Create generic geometry from WKB (ownership transferred to smart ptr)
    OGRGeometry *geom_raw = nullptr;
    OGRErr err = OGRGeometryFactory::createFromWkb(wkb_data, getLAMB93(),
                                                   &geom_raw, wkb_size);

    if (err != OGRERR_NONE || !geom_raw) {
        std::cerr << "Failed to parse WKB: " << err << std::endl;
        return nullptr;
    }

    return OGRGeometryPtr(geom_raw);
}

OGRMultiPolygonPtr
parseWKBToMultiPolygon(std::shared_ptr<arrow::Scalar> scalar_ptr) {
    OGRGeometryPtr geom = parseWKBToGeometry(scalar_ptr);

    if (!geom) {
        std::cerr << "Failed to parse WKB geometry" << std::endl;
        return nullptr;
    }

    // Check if it's a MultiPolygon (including 25D variants)
    OGRwkbGeometryType geom_type = wkbFlatten(geom->getGeometryType());
    if (geom_type == wkbMultiPolygon) {
        OGRMultiPolygonPtr multi_polygon(geom.release()->toMultiPolygon());
        return multi_polygon;
    }

    std::cerr << "Geometry is not MultiPolygon (type: " << geom_type << ")"
              << std::endl;
    return nullptr;
}

arrow::Status open_parquet(std::string &input_file) {
    std::cout << std::format("Reading {}...", input_file) << std::endl;
    arrow::MemoryPool *pool = arrow::default_memory_pool();
    std::shared_ptr<arrow::io::RandomAccessFile> input;
    ARROW_ASSIGN_OR_RAISE(input, arrow::io::ReadableFile::Open(input_file));

    std::cout << "Opening the reader..." << std::endl;
    // Open Parquet file reader
    std::unique_ptr<parquet::arrow::FileReader> arrow_reader;
    ARROW_ASSIGN_OR_RAISE(arrow_reader, parquet::arrow::OpenFile(input, pool));

    std::cout << "Reading into a table..." << std::endl;
    // Read entire file as a single Arrow table
    std::shared_ptr<arrow::Table> table;
    ARROW_RETURN_NOT_OK(arrow_reader->ReadTable(&table));

    std::cout << table->schema()->ToString() << std::endl;

    std::cout << "Schema:" << std::endl;
    auto schema = table->schema();
    int num_fields = schema->num_fields();
    int geom_field_idx = -1;
    for (int i = 0; i < num_fields; ++i) {
        auto field = schema->field(i);
        if (field->name() == "geometry") {
            geom_field_idx = i;
        }
        // std::cout << "  " << field->name() << ": " <<
        // field->type()->ToString()
        //           << std::endl;
    }
    // std::cout << "Number of rows: " << table->num_rows() << std::endl;
    // std::cout << "Number of columns: " << num_fields << std::endl;
    // std::cout << "Geometry column index: " << geom_field_idx << std::endl;

    // std::cout << "\nFirst 10 rows:" << std::endl;

    // std::vector<OGRGeometryPtr> geometries;
    // int64_t max_rows_to_print = 10;
    // int64_t num_rows_to_print = std::min(max_rows_to_print,
    // table->num_rows()); for (int64_t row = 0; row < num_rows_to_print; ++row)
    // {
    //     std::cout << "Row " << row << ": " << std::endl;
    //     for (int col = 0; col < num_fields; ++col) {
    //         auto field = schema->field(col);
    //         auto chunk = table->column(col)->chunk(
    //             0); // Assume single chunk for simplicity
    //         if (chunk->IsNull(row)) {
    //             std::cout << "NULL";
    //         } else {
    //             auto scalar = chunk->GetScalar(row);
    //             if (!scalar.ok()) {
    //                 std::cerr << "Not OK!" << std::endl;
    //             }
    //             auto scalar_ptr = scalar.ValueOrDie();

    //             if (col == geom_field_idx) {
    //                 auto geometry = parseWKBToGeometry(scalar_ptr);
    //                 std::cout << geometry->exportToWkt();
    //                 geometries.emplace_back(geometry.release());
    //             } else {
    //                 std::cout << scalar_ptr->ToString();
    //             }
    //         }
    //         if (col < num_fields - 1)
    //             std::cout << ", ";
    //     }
    //     std::cout << std::endl;
    // }

    return arrow::Status::OK();
}

arrow::Status read_building_outlines_from_bd_topo(
    const std::string &bd_topo_parquet_file,
    std::vector<MultiPolygonZWithAttributes> &outlines) {
    std::cout << std::format("Reading {}...", bd_topo_parquet_file)
              << std::endl;
    arrow::MemoryPool *pool = arrow::default_memory_pool();
    std::shared_ptr<arrow::io::RandomAccessFile> input;
    ARROW_ASSIGN_OR_RAISE(input,
                          arrow::io::ReadableFile::Open(bd_topo_parquet_file));

    // Open Parquet file reader
    std::cout << "Opening the reader..." << std::endl;
    std::unique_ptr<parquet::arrow::FileReader> arrow_reader;
    ARROW_ASSIGN_OR_RAISE(arrow_reader, parquet::arrow::OpenFile(input, pool));

    // Read entire file as a single Arrow table
    std::cout << "Reading into a table..." << std::endl;
    std::shared_ptr<arrow::Table> table;
    ARROW_RETURN_NOT_OK(arrow_reader->ReadTable(&table));

    // Find the geometry column and other relevant columns
    // std::cout << "Schema:" << std::endl;
    auto schema = table->schema();
    int num_fields = schema->num_fields();
    int geom_field_idx = -1;
    int id_field_idx = -1;
    int outline_source_field_idx = -1;
    for (int i = 0; i < num_fields; ++i) {
        auto field = schema->field(i);
        if (field->name() == "geometry") {
            geom_field_idx = i;
        } else if (field->name() == "cleabs") {
            id_field_idx = i;
        } else if (field->name() == "origine_du_batiment") {
            outline_source_field_idx = i;
        }
        // std::cout << "  " << field->name() << ": " <<
        // field->type()->ToString()
        //           << std::endl;
    }

    // Check that required columns are present
    if (geom_field_idx == -1) {
        return arrow::Status::Invalid(
            "Geometry column not found in Parquet file");
    }
    if (id_field_idx == -1) {
        return arrow::Status::Invalid("ID column not found in Parquet file");
    }
    if (outline_source_field_idx == -1) {
        return arrow::Status::Invalid(
            "Outline source column not found in Parquet file");
    }

    long num_rows = table->num_rows();
    std::cout << "Number of features: " << num_rows << std::endl;
    std::cout << "Number of columns: " << num_fields << std::endl;
    // std::cout << "Geometry column index: " << geom_field_idx << std::endl;
    // std::cout << "ID column index: " << id_field_idx << std::endl;
    // std::cout << "Outline source column index: " << outline_source_field_idx
    // << std::endl;

    // Read the geometries and attributes from the table and construct the
    // output
    ProgressBarTotal progress_bar(num_rows, "Processing building outlines");
    outlines.clear();
    for (std::size_t row = 0; row < num_rows; ++row) {
        auto geom_chunk = table->column(geom_field_idx)->chunk(0);
        auto id_chunk = table->column(id_field_idx)->chunk(0);
        auto outline_source_chunk =
            table->column(outline_source_field_idx)->chunk(0);

        if (geom_chunk->IsNull(row)) {
            std::cerr << "Warning: Geometry is null for row " << row
                      << ", skipping this feature." << std::endl;
            continue;
        }
        auto geom_scalar = geom_chunk->GetScalar(row);
        if (!geom_scalar.ok()) {
            std::cerr << "Warning: Failed to get geometry scalar for row "
                      << row << ", skipping this feature." << std::endl;
            continue;
        }
        auto geom_scalar_ptr = geom_scalar.ValueOrDie();
        // std::unique_ptr<geos::geom::Geometry> geometry =
        //     wkb_reader.hex_to_geometry(geom_scalar_ptr);
        auto multi_polygon = parseWKBToMultiPolygon(geom_scalar_ptr);

        std::string id = "unknown_id";
        if (!id_chunk->IsNull(row)) {
            auto id_scalar = id_chunk->GetScalar(row);
            if (id_scalar.ok()) {
                id = id_scalar.ValueOrDie()->ToString();
            }
        }

        OutlineSource::Id outline_source = OutlineSource::Id::Unknown;
        if (!outline_source_chunk->IsNull(row)) {
            auto outline_source_scalar = outline_source_chunk->GetScalar(row);
            if (outline_source_scalar.ok()) {
                std::string outline_source_str =
                    outline_source_scalar.ValueOrDie()->ToString();
                outline_source = OutlineSource::from_string(outline_source_str);
            }
        }

        try {
            outlines.push_back({std::move(multi_polygon), id, outline_source});
        } catch (const std::exception &e) {
            std::cerr << "Warning: Failed to process geometry for row " << row
                      << ": " << e.what() << ", skipping this feature."
                      << std::endl;
        }
        progress_bar.increment(1);
    }
    progress_bar.finish();

    return arrow::Status::OK();
}

std::string ProjjsonFromEPSG(const std::string &epsg) {
    OGRSpatialReference srs;
    if (srs.SetFromUserInput(epsg.c_str()) != OGRERR_NONE) {
        throw std::runtime_error("Failed to parse CRS: " + epsg);
    }
    char *projjson_raw = nullptr;
    if (srs.exportToPROJJSON(&projjson_raw, nullptr) != OGRERR_NONE) {
        throw std::runtime_error("Failed to export PROJJSON");
    }
    std::string projjson(projjson_raw);
    CPLFree(projjson_raw);
    return projjson;
}

char *buildGeoMetaData(std::string crs_epsg = "EPSG:2154",
                       std::string geometry_type = "MultiPolygon",
                       std::string primary_column = "geometry") {

    // Get PROJJSON from EPSG
    std::string projjson = ProjjsonFromEPSG(crs_epsg);

    // Build metadata object
    json metadata;
    metadata["version"] = "1.1.0";
    metadata["primary_column"] = primary_column;

    metadata["columns"] = json::object();
    json &col = metadata["columns"][primary_column];
    col["encoding"] = "WKB";
    col["geometry_types"] = json::array({geometry_type});
    col["crs"] = json::parse(projjson); // PROJJSON is already valid JSON
    col["covering"] = json::object();
    col["covering"]["bbox"]["xmin"] = json::array({"bbox", "xmin"});
    col["covering"]["bbox"]["ymin"] = json::array({"bbox", "ymin"});
    col["covering"]["bbox"]["xmax"] = json::array({"bbox", "xmax"});
    col["covering"]["bbox"]["ymax"] = json::array({"bbox", "ymax"});

    // Serialize to string and allocate char*
    std::string json_str = metadata.dump(2); // Pretty-print with 2 spaces
    char *result = (char *)CPLMalloc(json_str.size() + 1);
    std::strcpy(result, json_str.c_str());

    return result;
}

arrow::Status write_multi_polygons_to_parquet(
    const std::vector<MultiPolygonZWithAttributes> &multi_polygons,
    const std::string &output_file, bool overwrite) {
    // Inspired from
    // https://gist.github.com/jpswinski/13074fc773f92a529f98b274e5ad5283

    const int64_t max_row_group_length = 100000;
    const int64_t batch_size = 50000;

    if (std::filesystem::exists(output_file) && !overwrite) {
        throw std::runtime_error("Output file already exists: " + output_file);
    }

    // Build Schema
    std::vector<std::shared_ptr<arrow::Field>> schema_vector;
    schema_vector.push_back(arrow::field("cleabs", arrow::utf8()));
    schema_vector.push_back(arrow::field("origine_du_batiment", arrow::utf8()));
    schema_vector.push_back(arrow::field("geometry", arrow::binary()));
    auto schema = std::make_shared<arrow::Schema>(schema_vector);

    // Create Arrow Output Stream
    std::shared_ptr<arrow::io::FileOutputStream> file_output_stream;
    PARQUET_ASSIGN_OR_THROW(file_output_stream,
                            arrow::io::FileOutputStream::Open(output_file));

    // Create Writer Properties
    parquet::WriterProperties::Builder writer_props_builder;
    writer_props_builder.compression(parquet::Compression::ZSTD)
        ->max_row_group_length(max_row_group_length);
    std::shared_ptr<parquet::WriterProperties> writer_props =
        writer_props_builder.build();

    // Create Arrow Writer Properties
    auto arrow_writer_props =
        parquet::ArrowWriterProperties::Builder().store_schema()->build();

    // Build GeoParquet MetaData
    auto metadata = schema->metadata()
                        ? schema->metadata()->Copy()
                        : std::make_shared<arrow::KeyValueMetadata>();
    const char *metadata_str = buildGeoMetaData();
    metadata->Append("geo", metadata_str);
    schema = schema->WithMetadata(metadata);
    delete[] metadata_str;

    // Create Parquet Writer
    std::unique_ptr<parquet::arrow::FileWriter> parquet_writer;
    PARQUET_ASSIGN_OR_THROW(
        parquet_writer,
        parquet::arrow::FileWriter::Open(*schema, arrow::default_memory_pool(),
                                         file_output_stream, writer_props,
                                         arrow_writer_props));

    // Write data in batches
    ProgressBarTotal progress_bar(multi_polygons.size(),
                                  "Writing building outlines to Parquet");
    int64_t num_rows = multi_polygons.size();

    for (int64_t start = 0; start < num_rows; start += batch_size) {
        int64_t end = std::min(start + batch_size, num_rows);
        int64_t batch_rows = end - start;

        // Pre-allocate ALL WKB buffers first for this batch
        std::vector<std::vector<uint8_t>> wkb_buffers(batch_size);
        std::vector<size_t> wkb_sizes(batch_size);

        // Bulk WKB export (single pass through geometries)
        for (int64_t row = start; row < end; ++row) {
            int idx = row - start;
            OGRGeometry *geom = multi_polygons[row].multi_polygon.get();
            size_t wkb_size = geom->WkbSize();
            wkb_buffers[idx].resize(wkb_size);
            wkb_sizes[idx] = wkb_size;
            if (geom->exportToWkb(wkbNDR, wkb_buffers[idx].data()) !=
                OGRERR_NONE) {
                return arrow::Status::Invalid(
                    "Failed to export geometry to WKB for row " +
                    std::to_string(row));
            }
        }

        // Build arrays for each column in the batch
        std::vector<std::shared_ptr<arrow::Array>> columns(3);
        {
            arrow::StringBuilder builder;
            (void)builder.Reserve(batch_rows);

            // Total capacity for all strings in this batch
            size_t total_string_size = 0;
            for (int row = start; row < end; row++) {
                total_string_size += multi_polygons[row].id.size();
            }
            (void)builder.ReserveData(total_string_size);

            for (int row = start; row < end; row++) {
                builder.UnsafeAppend(multi_polygons[row].id);
            }
            ARROW_RETURN_NOT_OK(builder.Finish(&columns[0]));
        }

        {
            arrow::StringBuilder builder;
            (void)builder.Reserve(batch_rows);

            // Total capacity for all strings in this batch
            size_t total_string_size = 0;
            for (int row = start; row < end; row++) {
                total_string_size +=
                    OutlineSource::name(multi_polygons[row].outline_source)
                        .size();
            }
            (void)builder.ReserveData(total_string_size);

            for (int row = start; row < end; row++) {
                builder.UnsafeAppend(
                    OutlineSource::name(multi_polygons[row].outline_source));
            }
            ARROW_RETURN_NOT_OK(builder.Finish(&columns[1]));
        }

        {
            arrow::BinaryBuilder builder;
            (void)builder.Reserve(batch_rows);

            // Total capacity for all WKBs
            size_t total_wkb_size = 0;
            for (size_t size : wkb_sizes)
                total_wkb_size += size;
            (void)builder.ReserveData(
                total_wkb_size); // Pre-allocate data buffer

            for (int row = start; row < end; row++) {
                int idx = row - start;
                builder.UnsafeAppend(wkb_buffers[idx].data(), wkb_sizes[idx]);
            }
            ARROW_RETURN_NOT_OK(builder.Finish(&columns[2]));
        }

        auto batch = arrow::RecordBatch::Make(schema, batch_rows, columns);
        ARROW_RETURN_NOT_OK(parquet_writer->WriteRecordBatch(*batch));
        progress_bar.increment(batch_rows);
    }
    progress_bar.finish();

    // Close the writer
    ARROW_RETURN_NOT_OK(parquet_writer->Close());
    return arrow::Status::OK();
}

arrow::Status read_write_bd_topo(const std::string &input_parquet_file,
                                 const std::string &output_parquet_file,
                                 bool overwrite) {
    if (std::filesystem::exists(output_parquet_file) && !overwrite) {
        throw std::runtime_error("Output file already exists: " +
                                 output_parquet_file);
    }

    std::vector<MultiPolygonZWithAttributes> outlines;
    auto status =
        read_building_outlines_from_bd_topo(input_parquet_file, outlines);
    if (!status.ok()) {
        std::cerr << "Error reading BD TOPO: " << status.ToString()
                  << std::endl;
        return status;
    }
    std::cout << "Successfully read " << outlines.size()
              << " building outlines from BD TOPO." << std::endl;

    status = write_multi_polygons_to_parquet(outlines, output_parquet_file,
                                             overwrite);
    if (!status.ok()) {
        std::cerr << "Error writing output Parquet file: " << status.ToString()
                  << std::endl;
        return status;
    }
    std::cout << "Successfully wrote building outlines to "
              << output_parquet_file << "." << std::endl;

    return arrow::Status::OK();
}