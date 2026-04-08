#include "parquet.hpp"

#include <cstddef>
#include <cstdint>
#include <format>
#include <iostream>
#include <memory>
#include <ostream>
#include <string>
#include <sys/types.h>
#include <utility>

#include <arrow/api.h>
#include <arrow/array/builder_binary.h>
#include <arrow/array/builder_nested.h>
#include <arrow/io/file.h>
#include <arrow/scalar.h>
#include <arrow/status.h>
#include <arrow/type.h>
#include <arrow/type_fwd.h>
#include <parquet/arrow/reader.h>
#include <parquet/arrow/writer.h>

#include <ogr_geometry.h>
#include <ogr_spatialref.h>

#include "json/json.hpp"

#include "geometry.hpp"
#include "parquet/reader.hpp"
#include "utils/pbar.hpp"

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
    // num_rows = std::min(
    //     num_rows, static_cast<long>(10)); // Limit to 100k rows for testing
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

char *buildGeoMetaData(std::string crs_epsg, std::string geometry_type,
                       std::string primary_column) {

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

    const int64_t max_row_group_length = 500000;
    const int64_t batch_size = 50000;

    if (std::filesystem::exists(output_file) && !overwrite) {
        throw std::runtime_error("Output file already exists: " + output_file);
    }

    std::cout << std::format("Writing {} building outlines to {}...",
                             multi_polygons.size(), output_file)
              << std::endl;

    // Build Schema
    std::cout << "Building schema..." << std::endl;
    auto bbox_type = std::make_shared<arrow::StructType>(
        std::vector<std::shared_ptr<arrow::Field>>{
            arrow::field("xmin", arrow::float64()),
            arrow::field("ymin", arrow::float64()),
            arrow::field("xmax", arrow::float64()),
            arrow::field("ymax", arrow::float64())});
    std::vector<std::shared_ptr<arrow::Field>> schema_vector{
        arrow::field("cleabs", arrow::utf8()),
        arrow::field("origine_du_batiment", arrow::utf8()),
        arrow::field("geometry", arrow::binary()),
        arrow::field("bbox", bbox_type)};
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
    std::cout << "Building GeoParquet metadata..." << std::endl;
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
    std::cout << "Writing data in batches..." << std::endl;
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
        std::vector<std::shared_ptr<arrow::Array>> columns(
            schema_vector.size());
        {
            arrow::StringBuilder builder;
            (void)builder.Reserve(batch_rows);

            // Total capacity for all strings in this batch
            size_t total_string_size = 0;
            for (int row = start; row < end; row++) {
                total_string_size += multi_polygons[row].id.size();
            }
            (void)builder.ReserveData(total_string_size);

            // Append all strings in this batch
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

            // Append all strings in this batch
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
            (void)builder.ReserveData(total_wkb_size);

            // Append all WKBs in this batch
            for (int row = start; row < end; row++) {
                int idx = row - start;
                builder.UnsafeAppend(wkb_buffers[idx].data(), wkb_sizes[idx]);
            }
            ARROW_RETURN_NOT_OK(builder.Finish(&columns[2]));
        }

        {
            auto x_min_builder = std::make_shared<arrow::DoubleBuilder>();
            auto y_min_builder = std::make_shared<arrow::DoubleBuilder>();
            auto x_max_builder = std::make_shared<arrow::DoubleBuilder>();
            auto y_max_builder = std::make_shared<arrow::DoubleBuilder>();

            arrow::StructBuilder builder(
                bbox_type, arrow::default_memory_pool(),
                {x_min_builder, y_min_builder, x_max_builder, y_max_builder});

            (void)builder.Reserve(batch_rows);

            // Pre-allocate vectors for bbox values
            std::vector<double> x_mins, y_mins, x_maxs, y_maxs;
            x_mins.reserve(batch_rows);
            y_mins.reserve(batch_rows);
            x_maxs.reserve(batch_rows);
            y_maxs.reserve(batch_rows);

            // Extract all bbox values
            for (int row = start; row < end; row++) {
                OGREnvelopePtr bbox = multi_polygons[row].bounding_box();
                x_mins.push_back(bbox->MinX);
                y_mins.push_back(bbox->MinY);
                x_maxs.push_back(bbox->MaxX);
                y_maxs.push_back(bbox->MaxY);
            }

            // Bulk append values to each field
            (void)x_min_builder->AppendValues(x_mins);
            (void)y_min_builder->AppendValues(y_mins);
            (void)x_max_builder->AppendValues(x_maxs);
            (void)y_max_builder->AppendValues(y_maxs);

            // Finalize each struct
            for (int row = start; row < end; row++) {
                (void)builder.Append();
            }

            ARROW_RETURN_NOT_OK(builder.Finish(&columns[3]));
        }

        auto batch = arrow::RecordBatch::Make(schema, batch_rows, columns);
        ARROW_RETURN_NOT_OK(parquet_writer->WriteRecordBatch(*batch));
        progress_bar.increment(batch_rows);
    }
    progress_bar.finish();

    // Close the writer
    ARROW_RETURN_NOT_OK(parquet_writer->Close());
    std::cout << "Finished writing Parquet file." << std::endl;
    return arrow::Status::OK();
}

// Template function write_geoms_to_parquet implementation moved to parquet.hpp

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

// void get_value(const std::shared_ptr<arrow::Array> &chunk, std::size_t row,
//                std::shared_ptr<arrow::Scalar> &value) {
//     auto scalar = chunk->GetScalar(row);
//     if (!scalar.ok()) {
//         throw std::runtime_error("Failed to get scalar for row " +
//                                  std::to_string(row));
//     }
//     value = scalar.ValueOrDie();
// }

// void get_value_uint(const std::shared_ptr<arrow::Array> &chunk, std::size_t
// row,
//                     uint &value) {
//     std::shared_ptr<arrow::Scalar> scalar;
//     get_value(chunk, row, scalar);
//     if (!scalar->is_valid) {
//         throw std::runtime_error("Value is null for row " +
//                                  std::to_string(row));
//     }
//     // if (scalar->type->id() != arrow::Type::UINT64) {
//     //     throw std::runtime_error("Expected UINT64 type for row " +
//     //                              std::to_string(row));
//     // }
//     auto uint_scalar = std::static_pointer_cast<arrow::UInt32Scalar>(scalar);
//     std::cout << "Value for row " << row << ": " << uint_scalar->value
//               << std::endl;
//     value = uint_scalar->value;
// }

// void get_value_double(const std::shared_ptr<arrow::Array> &chunk,
//                       std::size_t row, double &value) {
//     std::shared_ptr<arrow::Scalar> scalar;
//     get_value(chunk, row, scalar);
//     if (!scalar->is_valid) {
//         throw std::runtime_error("Value is null for row " +
//                                  std::to_string(row));
//     }
//     auto double_scalar =
//     std::static_pointer_cast<arrow::DoubleScalar>(scalar); std::cout <<
//     "Value for row " << row << ": " << double_scalar->value
//               << std::endl;
//     value = double_scalar->value;
// }

// void get_value_string(const std::shared_ptr<arrow::Array> &chunk,
//                       std::size_t row, std::string &value) {
//     std::shared_ptr<arrow::Scalar> scalar;
//     get_value(chunk, row, scalar);
//     value = scalar->ToString();
// }

// arrow::Status _old_read_bd_topo_as_grouped_edges(
//     const std::string &edges_parquet_file,
//     const std::string &intersections_parquet_file,
//     std::vector<BDTOPOEdge> &edges,
//     std::vector<std::pair<std::size_t, std::size_t>> &intersections) {
//     std::cout << std::format("Reading {}...", edges_parquet_file) <<
//     std::endl; arrow::MemoryPool *pool = arrow::default_memory_pool();
//     std::shared_ptr<arrow::io::RandomAccessFile> input;
//     ARROW_ASSIGN_OR_RAISE(input,
//                           arrow::io::ReadableFile::Open(edges_parquet_file));

//     // Open Parquet file reader
//     std::cout << "Opening the reader..." << std::endl;
//     std::unique_ptr<parquet::arrow::FileReader> arrow_reader;
//     ARROW_ASSIGN_OR_RAISE(arrow_reader, parquet::arrow::OpenFile(input,
//     pool));

//     // Read entire file as a single Arrow table
//     std::cout << "Reading into a table..." << std::endl;
//     std::shared_ptr<arrow::Table> table;
//     ARROW_RETURN_NOT_OK(arrow_reader->ReadTable(&table));

//     // Find the geometry column and other relevant columns
//     // std::cout << "Schema:" << std::endl;
//     auto schema = table->schema();
//     int num_fields = schema->num_fields();
//     int building_id_field_idx = -1;
//     int polygon_idx_field_idx = -1;
//     int ring_idx_field_idx = -1;
//     int edge_idx_field_idx = -1;
//     int edge_id_field_idx = -1;
//     int start_point_field_idx = -1;
//     int end_point_field_idx = -1;
//     for (int i = 0; i < num_fields; ++i) {
//         auto field = schema->field(i);
//         if (field->name() == "cleabs") {
//             building_id_field_idx = i;
//         } else if (field->name() == "idx_polygon") {
//             polygon_idx_field_idx = i;
//         } else if (field->name() == "idx_ring") {
//             ring_idx_field_idx = i;
//         } else if (field->name() == "idx_edge") {
//             edge_idx_field_idx = i;
//         } else if (field->name() == "edge_id") {
//             edge_id_field_idx = i;
//         } else if (field->name() == "start_point") {
//             start_point_field_idx = i;
//         } else if (field->name() == "end_point") {
//             end_point_field_idx = i;
//         }
//     }

//     // Check that required columns are present
//     if (building_id_field_idx == -1) {
//         return arrow::Status::Invalid(
//             "Building ID column not found in Parquet file");
//     }
//     if (polygon_idx_field_idx == -1) {
//         return arrow::Status::Invalid(
//             "Polygon index column not found in Parquet file");
//     }
//     if (ring_idx_field_idx == -1) {
//         return arrow::Status::Invalid(
//             "Ring index column not found in Parquet file");
//     }
//     if (edge_idx_field_idx == -1) {
//         return arrow::Status::Invalid(
//             "Edge index column not found in Parquet file");
//     }
//     if (edge_id_field_idx == -1) {
//         return arrow::Status::Invalid(
//             "Edge ID column not found in Parquet file");
//     }
//     if (start_point_field_idx == -1) {
//         return arrow::Status::Invalid(
//             "Start point column not found in Parquet file");
//     }
//     if (end_point_field_idx == -1) {
//         return arrow::Status::Invalid(
//             "End point column not found in Parquet file");
//     }

//     long num_rows = table->num_rows();
//     // num_rows = std::min(
//     //     num_rows, static_cast<long>(10)); // Limit to 100k rows for
//     testing std::cout << "Number of features: " << num_rows << std::endl;
//     std::cout << "Number of columns: " << num_fields << std::endl;

//     // Read the geometries and attributes from the table and construct the
//     // output
//     ProgressBarTotal progress_bar(num_rows, "Processing building outlines");
//     edges.clear();
//     for (std::size_t row = 0; row < num_rows; ++row) {
//         auto building_id_chunk =
//         table->column(building_id_field_idx)->chunk(0); auto
//         polygon_idx_chunk = table->column(polygon_idx_field_idx)->chunk(0);
//         auto ring_idx_chunk = table->column(ring_idx_field_idx)->chunk(0);
//         auto edge_idx_chunk = table->column(edge_idx_field_idx)->chunk(0);
//         auto edge_id_chunk = table->column(edge_id_field_idx)->chunk(0);
//         auto start_point_chunk =
//         table->column(start_point_field_idx)->chunk(0); auto end_point_chunk
//         = table->column(end_point_field_idx)->chunk(0);

//         // Extract attributes
//         std::string building_id;
//         uint polygon_idx;
//         uint ring_idx;
//         uint edge_idx;
//         uint edge_id;
//         // std::string outline_source_str;
//         std::shared_ptr<arrow::Scalar> start_point_scalar;
//         std::shared_ptr<arrow::Scalar> end_point_scalar;

//         // TODO: Figure out how to read the geometry attributes

//         get_value_string(building_id_chunk, row, building_id);
//         get_value_uint(polygon_idx_chunk, row, polygon_idx);
//         get_value_uint(ring_idx_chunk, row, ring_idx);
//         get_value_uint(edge_idx_chunk, row, edge_idx);
//         get_value_uint(edge_id_chunk, row, edge_id);
//         // get_value_string(outline_source_chunk, row, outline_source_str);
//         // OutlineSource::Id outline_source =
//         //     OutlineSource::from_string(outline_source_str);
//         get_value(start_point_chunk, row, start_point_scalar);
//         std::cout << "Row " << row << ": start_point_scalar type = "
//                   << start_point_scalar->type->ToString() << std::endl;
//         std::cout << "Row " << row << ": start_point_scalar value = "
//                   << start_point_scalar->ToString() << std::endl;

//         try {
//             edges.push_back({
//                 building_id,
//                 polygon_idx,
//                 ring_idx,
//                 edge_idx,
//                 edge_id,
//                 // outline_source,
//             });
//         } catch (const std::exception &e) {
//             std::cerr << "Warning: Failed to process geometry for row " <<
//             row
//                       << ": " << e.what() << ", skipping this feature."
//                       << std::endl;
//         }
//         progress_bar.increment(1);
//     }
//     progress_bar.finish();

//     return arrow::Status::OK();
// }

arrow::Status read_bd_topo_as_grouped_edges(
    const std::string &edges_parquet_file,
    const std::string &intersections_parquet_file,
    std::vector<BDTOPOEdge> &edges,
    std::vector<std::pair<uint32_t, uint32_t>> &intersections) {
    arrow::Status status;

    // Prepare the columns to read from the edges Parquet file
    std::vector<RequestedColumn> edges_columns{
        {"cleabs", ParquetValueType::Utf8},
        {"idx_polygon", ParquetValueType::UInt8},
        {"idx_ring", ParquetValueType::UInt8},
        {"idx_edge", ParquetValueType::UInt16},
        {"edge_key", ParquetValueType::UInt32},
        {"start_x", ParquetValueType::Float64},
        {"start_y", ParquetValueType::Float64},
        {"start_z", ParquetValueType::Float64},
        {"end_x", ParquetValueType::Float64},
        {"end_y", ParquetValueType::Float64},
        {"end_z", ParquetValueType::Float64},
    };

    // Read the edges data from the Parquet file using the ParquetReader
    ParquetReader edges_reader(edges_parquet_file);
    GenericParquetOutput edges_output;
    status = edges_reader.read_columns(edges_columns, edges_output);
    if (!status.ok()) {
        std::cerr << "Error reading edges Parquet file: " << status.ToString()
                  << std::endl;
        return status;
    }

    // Convert the input data into the desired BDTOPOEdge format
    edges.reserve(edges_output.row_count);
    std::map<uint32_t, std::size_t> edge_key_to_index;
    for (std::size_t i = 0; i < edges_output.row_count; ++i) {
        edges.push_back({
            edges_output.value<std::string>("cleabs", i),
            edges_output.value<uint8_t>("idx_polygon", i),
            edges_output.value<uint8_t>("idx_ring", i),
            edges_output.value<uint16_t>("idx_edge", i),
            edges_output.value<uint32_t>("edge_key", i),
            {edges_output.value<double>("start_x", i),
             edges_output.value<double>("start_y", i),
             edges_output.value<double>("start_z", i)},
            {edges_output.value<double>("end_x", i),
             edges_output.value<double>("end_y", i),
             edges_output.value<double>("end_z", i)},
        });
        // std::cout << "Read row " << i << ": cleabs=" <<
        // edges.back().building_id
        //           << ", idx_polygon="
        //           << static_cast<int>(edges.back().polygon_idx)
        //           << ", idx_ring=" << static_cast<int>(edges.back().ring_idx)
        //           << ", idx_edge=" << edges.back().edge_idx
        //           << ", edge_key=" << edges.back().edge_key << ",
        //           start_point=("
        //           << edges.back().start.x() << ", " << edges.back().start.y()
        //           << ", " << edges.back().start.z() << ")"
        //           << ", end_point=(" << edges.back().end.x() << ", "
        //           << edges.back().end.y() << ", " << edges.back().end.z() <<
        //           ")"
        //           << std::endl;
        edge_key_to_index[edges.back().edge_key] = i;
    }

    // Prepare the columns to read from the intersections Parquet file
    std::vector<RequestedColumn> intersection_columns{
        {"edge_key_a", ParquetValueType::UInt32},
        {"edge_key_b", ParquetValueType::UInt32},
    };

    // Read the intersections data from the Parquet file using the ParquetReader
    ParquetReader intersections_reader(intersections_parquet_file);
    GenericParquetOutput intersections_output;
    status = intersections_reader.read_columns(intersection_columns,
                                               intersections_output);
    if (!status.ok()) {
        std::cerr << "Error reading intersections Parquet file: "
                  << status.ToString() << std::endl;
        return status;
    }

    // Convert the input data into the desired format
    intersections.clear();
    intersections.reserve(intersections_output.row_count);
    for (std::size_t i = 0; i < intersections_output.row_count; ++i) {
        uint32_t edge_key_a =
            intersections_output.value<uint32_t>("edge_key_a", i);
        uint32_t edge_key_b =
            intersections_output.value<uint32_t>("edge_key_b", i);
        if (edge_key_to_index.count(edge_key_a) == 0) {
            std::cerr << "Warning: edge_key_a " << edge_key_a
                      << " not found in edges data, skipping this intersection."
                      << std::endl;
            continue;
        }
        if (edge_key_to_index.count(edge_key_b) == 0) {
            std::cerr << "Warning: edge_key_b " << edge_key_b
                      << " not found in edges data, skipping this intersection."
                      << std::endl;
            continue;
        }
        intersections.emplace_back(edge_key_to_index.at(edge_key_a),
                                   edge_key_to_index.at(edge_key_b));
    }

    return status;
}