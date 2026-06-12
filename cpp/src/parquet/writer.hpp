#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <format>
#include <iostream>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include <arrow/api.h>
#include <arrow/array/builder_binary.h>
#include <arrow/array/builder_nested.h>
#include <arrow/io/file.h>
#include <arrow/status.h>
#include <arrow/type.h>
#include <parquet/arrow/writer.h>

#include <ogr_api.h>
#include <ogr_geometry.h>
#include <ogr_spatialref.h>

#include "../geom/ogc_simple_features.hpp"
#include "../utils/pbar.hpp"

// Forward declaration for function used by template
char *buildGeoMetaData(std::string crs_epsg = "EPSG:2154",
                       std::string geometry_type = "MultiPolygon",
                       std::string primary_column = "geometry");

// Template function to write any geometry type that inherits from
// GeometryWithAttributes
template <typename GeomType>
arrow::Status write_geoms_to_parquet(const std::vector<GeomType> &geoms,
                                     const std::string &output_file,
                                     bool overwrite,
                                     std::string crs_epsg = "EPSG:2154",
                                     std::string geometry_type = "Unknown");

// Template implementation (must be in header)
template <typename GeomType>
arrow::Status write_geoms_to_parquet(const std::vector<GeomType> &geoms,
                                     const std::string &output_file,
                                     bool overwrite, std::string crs_epsg,
                                     std::string geometry_type) {
    // Ensure GeomType derives from GeometryWithAttributes
    static_assert(std::is_base_of<GeometryWithAttributes, GeomType>::value,
                  "GeomType must inherit from GeometryWithAttributes");

    const int64_t max_row_group_length = 500000;
    const int64_t batch_size = 50000;

    if (std::filesystem::exists(output_file) && !overwrite) {
        throw std::runtime_error("Output file already exists: " + output_file);
    }

    std::cout << std::format("Writing {} geometries to {}...", geoms.size(),
                             output_file)
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
    char *metadata_str = buildGeoMetaData(crs_epsg, geometry_type, "geometry");
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
    ProgressBarTotal progress_bar(geoms.size(),
                                  "Writing geometries to Parquet");
    int64_t num_rows = geoms.size();

    for (int64_t start = 0; start < num_rows; start += batch_size) {
        int64_t end = std::min(start + batch_size, num_rows);
        int64_t batch_rows = end - start;

        // Pre-allocate ALL WKB buffers first for this batch
        std::vector<std::vector<uint8_t>> wkb_buffers(batch_size);
        std::vector<size_t> wkb_sizes(batch_size);

        // Bulk WKB export (single pass through geometries)
        for (int64_t row = start; row < end; ++row) {
            int idx = row - start;
            OGRGeometryPtr geom = geoms[row].get_geom();
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
                total_string_size += geoms[row].get_id().size();
            }
            (void)builder.ReserveData(total_string_size);

            // Append all strings in this batch
            for (int row = start; row < end; row++) {
                builder.UnsafeAppend(geoms[row].get_id());
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
                    OutlineSource::name(geoms[row].get_outline_source()).size();
            }
            (void)builder.ReserveData(total_string_size);

            // Append all strings in this batch
            for (int row = start; row < end; row++) {
                builder.UnsafeAppend(
                    OutlineSource::name(geoms[row].get_outline_source()));
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
                OGREnvelopePtr bbox = geoms[row].bounding_box();
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