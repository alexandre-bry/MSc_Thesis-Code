#include "parquet.hpp"

#include <algorithm>
#include <arrow/scalar.h>
#include <cstddef>
#include <format>
#include <geom/Geometry.h>
#include <geom/Polygon.h>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "arrow/api.h"
#include "arrow/io/file.h"
#include "arrow/status.h"
#include "parquet/arrow/reader.h"

#include <geos/geom/GeometryFactory.h>
#include <geos/io/WKBReader.h>

struct WKBReader {
    geos::io::WKBReader reader;

    WKBReader()
        : reader(geos::io::WKBReader(*geos::geom::GeometryFactory::create())) {
          };

    std::unique_ptr<geos::geom::Geometry>
    hex_to_geometry(std::shared_ptr<arrow::Scalar> scalar_ptr) {
        auto [wkb_bytes, wkb_size_t] = process_scalar_ptr(scalar_ptr);
        return reader.read(wkb_bytes, wkb_size_t);
    }

    std::string hex_to_wkt(std::shared_ptr<arrow::Scalar> scalar_ptr) {
        auto [wkb_bytes, wkb_size_t] = process_scalar_ptr(scalar_ptr);
        return reader.read(wkb_bytes, wkb_size_t)->toString();
    }

  private:
    std::pair<const uint8_t *, std::size_t>
    process_scalar_ptr(std::shared_ptr<arrow::Scalar> scalar_ptr) {
        auto binary_scalar =
            std::static_pointer_cast<arrow::BinaryScalar>(scalar_ptr);

        const uint8_t *wkb_bytes =
            static_cast<const uint8_t *>(binary_scalar->data());
        int64_t wkb_size = binary_scalar->value->size();
        std::size_t wkb_size_t = static_cast<std::size_t>(wkb_size);
        return {wkb_bytes, wkb_size_t};
    }
};

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

    std::cout << "Schema:" << std::endl;
    auto schema = table->schema();
    int num_fields = schema->num_fields();
    int geom_field_idx = -1;
    for (int i = 0; i < num_fields; ++i) {
        auto field = schema->field(i);
        if (field->name() == "geometry") {
            geom_field_idx = i;
        }
        std::cout << "  " << field->name() << ": " << field->type()->ToString()
                  << std::endl;
    }
    std::cout << "Number of rows: " << table->num_rows() << std::endl;
    std::cout << "Number of columns: " << num_fields << std::endl;
    std::cout << "Geometry column index: " << geom_field_idx << std::endl;

    std::cout << "\nFirst 10 rows:" << std::endl;
    WKBReader wkb_reader;

    std::vector<std::unique_ptr<geos::geom::Geometry>> geometries;
    int64_t max_rows_to_print = 10;
    int64_t num_rows_to_print = std::min(max_rows_to_print, table->num_rows());
    for (int64_t row = 0; row < num_rows_to_print; ++row) {
        std::cout << "Row " << row << ": ";
        for (int col = 0; col < num_fields; ++col) {
            auto field = schema->field(col);
            auto chunk = table->column(col)->chunk(
                0); // Assume single chunk for simplicity
            if (chunk->IsNull(row)) {
                std::cout << "NULL";
            } else {
                auto scalar = chunk->GetScalar(row);
                auto scalar_ptr = scalar.ValueOrDie();

                if (col == geom_field_idx) {
                    std::cout << wkb_reader.hex_to_wkt(scalar_ptr);
                    geometries.emplace_back(
                        wkb_reader.hex_to_geometry(scalar_ptr));
                } else {
                    std::cout << scalar_ptr->ToString();
                }
            }
            if (col < num_fields - 1)
                std::cout << ", ";
        }
        std::cout << std::endl;
    }

    for (const auto &geometry : geometries) {
        auto coords = geometry->getCoordinates();
        auto coords_size = coords->getSize();
        for (std::size_t coord_idx = 0; coord_idx < coords->getSize();
             coord_idx++) {
            std::cout << coords->getAt(coord_idx);
            if (coord_idx < coords_size) {
                std::cout << ",";
            }
        }
        std::cout << std::endl;
    }

    return arrow::Status::OK();
}