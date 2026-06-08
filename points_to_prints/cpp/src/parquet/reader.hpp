#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include <arrow/api.h>

#include "../geom/cgal.hpp"

enum class ParquetValueType {
    Int32,
    Int64,
    UInt8,
    UInt16,
    UInt32,
    UInt64,
    Float32,
    Float64,
    Bool,
    Utf8,
    Binary,
};

struct RequestedColumn {
    std::string name;
    ParquetValueType expected_type;
    bool nullable = true;
};

using TypedColumnData =
    std::variant<std::vector<int32_t>, std::vector<int64_t>,
                 std::vector<uint8_t>, std::vector<uint16_t>,
                 std::vector<uint32_t>, std::vector<uint64_t>,
                 std::vector<float>, std::vector<double>, std::vector<bool>,
                 std::vector<std::string>, std::vector<std::vector<uint8_t>>>;

using ParquetValue =
    std::variant<int32_t, int64_t, uint8_t, uint16_t, uint32_t, uint64_t, float,
                 double, bool, std::string, std::vector<uint8_t>>;

struct GenericParquetOutput {
    std::map<std::string, std::size_t> dimension_to_index;
    std::vector<std::string> dimensions;
    std::vector<TypedColumnData> columns;
    std::vector<std::vector<bool>> null_masks;
    std::size_t row_count = 0;

    bool has_dimension(const std::string &dimension) const;
    std::size_t dimension_index(const std::string &dimension) const;
    bool value_is_null(const std::string &dimension,
                       std::size_t row_index) const;
    ParquetValue value_as_variant(const std::string &dimension,
                                  std::size_t row_index) const;

    template <typename T>
    auto value(const std::string &dimension, std::size_t row_index) const ->
        typename std::vector<T>::const_reference {
        std::size_t dim_idx = dimension_index(dimension);
        if (row_index >= row_count) {
            throw std::out_of_range("Row index out of bounds: " +
                                    std::to_string(row_index));
        }
        if (null_masks[dim_idx][row_index]) {
            throw std::runtime_error("Value is null for dimension '" +
                                     dimension + "' at row " +
                                     std::to_string(row_index));
        }

        auto typed_column = std::get_if<std::vector<T>>(&columns[dim_idx]);
        if (!typed_column) {
            throw std::runtime_error(
                "Requested C++ type does not match stored column type for '" +
                dimension + "'");
        }
        return (*typed_column)[row_index];
    }
};

struct ParquetReader {
    std::string input_file;

    explicit ParquetReader(const std::string &input_file);

    arrow::Status read_table(std::shared_ptr<arrow::Table> &table) const;

    arrow::Status
    read_columns(const std::vector<RequestedColumn> &requested_columns,
                 GenericParquetOutput &output) const;
};

struct BDTOPOEdge {
    std::string building_id;
    uint8_t polygon_idx;
    uint8_t ring_idx;
    uint16_t edge_idx;
    uint32_t edge_key;
    Point_3 start;
    Point_3 end;
};

arrow::Status read_bd_topo_as_grouped_edges(
    const std::string &edges_parquet_file,
    const std::string &intersections_parquet_file,
    std::vector<BDTOPOEdge> &edges,
    std::vector<std::pair<uint32_t, uint32_t>> &intersections);