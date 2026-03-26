#include "reader.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <arrow/api.h>
#include <arrow/io/file.h>
#include <arrow/type.h>
#include <parquet/arrow/reader.h>

namespace {

arrow::Type::type arrow_type_id_for(ParquetValueType type) {
    switch (type) {
    case ParquetValueType::Int32:
        return arrow::Type::INT32;
    case ParquetValueType::Int64:
        return arrow::Type::INT64;
    case ParquetValueType::UInt8:
        return arrow::Type::UINT8;
    case ParquetValueType::UInt16:
        return arrow::Type::UINT16;
    case ParquetValueType::UInt32:
        return arrow::Type::UINT32;
    case ParquetValueType::UInt64:
        return arrow::Type::UINT64;
    case ParquetValueType::Float32:
        return arrow::Type::FLOAT;
    case ParquetValueType::Float64:
        return arrow::Type::DOUBLE;
    case ParquetValueType::Bool:
        return arrow::Type::BOOL;
    case ParquetValueType::Utf8:
        return arrow::Type::STRING;
    case ParquetValueType::Binary:
        return arrow::Type::BINARY;
    }
    throw std::runtime_error("Unsupported expected parquet value type");
}

bool type_is_compatible(ParquetValueType expected, arrow::Type::type actual) {
    if (actual == arrow_type_id_for(expected)) {
        return true;
    }

    switch (expected) {
    case ParquetValueType::Int32:
        return actual == arrow::Type::INT64;
    case ParquetValueType::UInt8:
        return actual == arrow::Type::UINT16 || actual == arrow::Type::UINT32 ||
               actual == arrow::Type::UINT64;
    case ParquetValueType::UInt16:
        return actual == arrow::Type::UINT32 || actual == arrow::Type::UINT64;
    case ParquetValueType::UInt32:
        return actual == arrow::Type::UINT64;
    case ParquetValueType::Utf8:
        return actual == arrow::Type::LARGE_STRING;
    case ParquetValueType::Binary:
        return actual == arrow::Type::LARGE_BINARY;
    default:
        return false;
    }
}

const char *expected_type_name(ParquetValueType expected) {
    switch (expected) {
    case ParquetValueType::Int32:
        return "int32";
    case ParquetValueType::Int64:
        return "int64";
    case ParquetValueType::UInt8:
        return "uint8";
    case ParquetValueType::UInt16:
        return "uint16";
    case ParquetValueType::UInt32:
        return "uint32";
    case ParquetValueType::UInt64:
        return "uint64";
    case ParquetValueType::Float32:
        return "float32";
    case ParquetValueType::Float64:
        return "float64";
    case ParquetValueType::Bool:
        return "bool";
    case ParquetValueType::Utf8:
        return "utf8";
    case ParquetValueType::Binary:
        return "binary";
    }
    return "unknown";
}

TypedColumnData make_storage(ParquetValueType type, std::size_t capacity) {
    switch (type) {
    case ParquetValueType::Int32: {
        std::vector<int32_t> out;
        out.reserve(capacity);
        return out;
    }
    case ParquetValueType::Int64: {
        std::vector<int64_t> out;
        out.reserve(capacity);
        return out;
    }
    case ParquetValueType::UInt8: {
        std::vector<uint8_t> out;
        out.reserve(capacity);
        return out;
    }
    case ParquetValueType::UInt16: {
        std::vector<uint16_t> out;
        out.reserve(capacity);
        return out;
    }
    case ParquetValueType::UInt32: {
        std::vector<uint32_t> out;
        out.reserve(capacity);
        return out;
    }
    case ParquetValueType::UInt64: {
        std::vector<uint64_t> out;
        out.reserve(capacity);
        return out;
    }
    case ParquetValueType::Float32: {
        std::vector<float> out;
        out.reserve(capacity);
        return out;
    }
    case ParquetValueType::Float64: {
        std::vector<double> out;
        out.reserve(capacity);
        return out;
    }
    case ParquetValueType::Bool: {
        std::vector<bool> out;
        out.reserve(capacity);
        return out;
    }
    case ParquetValueType::Utf8: {
        std::vector<std::string> out;
        out.reserve(capacity);
        return out;
    }
    case ParquetValueType::Binary: {
        std::vector<std::vector<uint8_t>> out;
        out.reserve(capacity);
        return out;
    }
    }
    throw std::runtime_error("Unsupported expected parquet value type");
}

template <typename T>
void append_or_default(std::vector<T> &out, bool is_null, const T &value) {
    if (is_null) {
        out.emplace_back(T{});
    } else {
        out.emplace_back(value);
    }
}

} // namespace

bool GenericParquetOutput::has_dimension(const std::string &dimension) const {
    return dimension_to_index.contains(dimension);
}

std::size_t
GenericParquetOutput::dimension_index(const std::string &dimension) const {
    auto it = dimension_to_index.find(dimension);
    if (it == dimension_to_index.end()) {
        throw std::out_of_range("Unknown dimension: " + dimension);
    }
    return it->second;
}

bool GenericParquetOutput::value_is_null(const std::string &dimension,
                                         std::size_t row_index) const {
    std::size_t dim_idx = dimension_index(dimension);
    if (row_index >= row_count) {
        throw std::out_of_range("Row index out of bounds: " +
                                std::to_string(row_index));
    }
    return null_masks[dim_idx][row_index];
}

ParquetValue
GenericParquetOutput::value_as_variant(const std::string &dimension,
                                       std::size_t row_index) const {
    std::size_t dim_idx = dimension_index(dimension);
    if (row_index >= row_count) {
        throw std::out_of_range("Row index out of bounds: " +
                                std::to_string(row_index));
    }
    if (null_masks[dim_idx][row_index]) {
        throw std::runtime_error("Value is null for dimension '" + dimension +
                                 "' at row " + std::to_string(row_index));
    }

    const TypedColumnData &column_data = columns[dim_idx];
    if (auto typed = std::get_if<std::vector<int32_t>>(&column_data)) {
        return (*typed)[row_index];
    }
    if (auto typed = std::get_if<std::vector<int64_t>>(&column_data)) {
        return (*typed)[row_index];
    }
    if (auto typed = std::get_if<std::vector<uint32_t>>(&column_data)) {
        return (*typed)[row_index];
    }
    if (auto typed = std::get_if<std::vector<uint64_t>>(&column_data)) {
        return (*typed)[row_index];
    }
    if (auto typed = std::get_if<std::vector<float>>(&column_data)) {
        return (*typed)[row_index];
    }
    if (auto typed = std::get_if<std::vector<double>>(&column_data)) {
        return (*typed)[row_index];
    }
    if (auto typed = std::get_if<std::vector<bool>>(&column_data)) {
        return static_cast<bool>((*typed)[row_index]);
    }
    if (auto typed = std::get_if<std::vector<std::string>>(&column_data)) {
        return (*typed)[row_index];
    }
    if (auto typed =
            std::get_if<std::vector<std::vector<uint8_t>>>(&column_data)) {
        return (*typed)[row_index];
    }

    throw std::runtime_error("Unsupported stored column type");
}

ParquetReader::ParquetReader(const std::string &input_file)
    : input_file(input_file) {}

arrow::Status
ParquetReader::read_table(std::shared_ptr<arrow::Table> &table) const {
    arrow::MemoryPool *pool = arrow::default_memory_pool();
    std::shared_ptr<arrow::io::RandomAccessFile> input;
    ARROW_ASSIGN_OR_RAISE(input, arrow::io::ReadableFile::Open(input_file));

    std::unique_ptr<parquet::arrow::FileReader> arrow_reader;
    ARROW_ASSIGN_OR_RAISE(arrow_reader, parquet::arrow::OpenFile(input, pool));

    ARROW_RETURN_NOT_OK(arrow_reader->ReadTable(&table));
    return arrow::Status::OK();
}

arrow::Status ParquetReader::read_columns(
    const std::vector<RequestedColumn> &requested_columns,
    GenericParquetOutput &output) const {
    std::shared_ptr<arrow::Table> table;
    ARROW_RETURN_NOT_OK(read_table(table));

    output.dimension_to_index.clear();
    output.dimensions.clear();
    output.columns.clear();
    output.null_masks.clear();
    output.row_count = static_cast<std::size_t>(table->num_rows());

    output.dimensions.reserve(requested_columns.size());
    output.columns.reserve(requested_columns.size());
    output.null_masks.reserve(requested_columns.size());

    for (const RequestedColumn &request : requested_columns) {
        int field_idx = table->schema()->GetFieldIndex(request.name);
        if (field_idx < 0) {
            return arrow::Status::Invalid("Requested column not found: " +
                                          request.name);
        }

        const auto &field = table->schema()->field(field_idx);
        arrow::Type::type actual_type = field->type()->id();
        if (!type_is_compatible(request.expected_type, actual_type)) {
            return arrow::Status::TypeError(
                "Type mismatch for column '" + request.name + "'. Expected " +
                std::string(expected_type_name(request.expected_type)) +
                ", got " + field->type()->ToString());
        }

        std::size_t out_idx = output.dimensions.size();
        output.dimension_to_index[request.name] = out_idx;
        output.dimensions.push_back(request.name);
        output.columns.emplace_back(
            make_storage(request.expected_type, output.row_count));
        output.null_masks.emplace_back();
        output.null_masks.back().reserve(output.row_count);

        auto chunked_array = table->column(field_idx);
        TypedColumnData &storage = output.columns.back();
        std::vector<bool> &null_mask = output.null_masks.back();

        for (const auto &chunk : chunked_array->chunks()) {
            switch (request.expected_type) {
            case ParquetValueType::Int32: {
                auto values =
                    std::static_pointer_cast<arrow::Int32Array>(chunk);
                auto &out = std::get<std::vector<int32_t>>(storage);
                for (int64_t i = 0; i < values->length(); ++i) {
                    bool is_null = values->IsNull(i);
                    if (is_null && !request.nullable) {
                        return arrow::Status::Invalid(
                            "Non-nullable column contains null value: " +
                            request.name);
                    }
                    null_mask.push_back(is_null);
                    append_or_default(out, is_null, values->Value(i));
                }
                break;
            }
            case ParquetValueType::Int64: {
                auto values =
                    std::static_pointer_cast<arrow::Int64Array>(chunk);
                auto &out = std::get<std::vector<int64_t>>(storage);
                for (int64_t i = 0; i < values->length(); ++i) {
                    bool is_null = values->IsNull(i);
                    if (is_null && !request.nullable) {
                        return arrow::Status::Invalid(
                            "Non-nullable column contains null value: " +
                            request.name);
                    }
                    null_mask.push_back(is_null);
                    append_or_default(out, is_null, values->Value(i));
                }
                break;
            }
            case ParquetValueType::UInt8: {
                auto values =
                    std::static_pointer_cast<arrow::UInt8Array>(chunk);
                auto &out = std::get<std::vector<uint8_t>>(storage);
                for (int64_t i = 0; i < values->length(); ++i) {
                    bool is_null = values->IsNull(i);
                    if (is_null && !request.nullable) {
                        return arrow::Status::Invalid(
                            "Non-nullable column contains null value: " +
                            request.name);
                    }
                    null_mask.push_back(is_null);
                    append_or_default(out, is_null, values->Value(i));
                }
                break;
            }
            case ParquetValueType::UInt16: {
                auto values =
                    std::static_pointer_cast<arrow::UInt16Array>(chunk);
                auto &out = std::get<std::vector<uint16_t>>(storage);
                for (int64_t i = 0; i < values->length(); ++i) {
                    bool is_null = values->IsNull(i);
                    if (is_null && !request.nullable) {
                        return arrow::Status::Invalid(
                            "Non-nullable column contains null value: " +
                            request.name);
                    }
                    null_mask.push_back(is_null);
                    append_or_default(out, is_null, values->Value(i));
                }
                break;
            }
            case ParquetValueType::UInt32: {
                auto values =
                    std::static_pointer_cast<arrow::UInt32Array>(chunk);
                auto &out = std::get<std::vector<uint32_t>>(storage);
                for (int64_t i = 0; i < values->length(); ++i) {
                    bool is_null = values->IsNull(i);
                    if (is_null && !request.nullable) {
                        return arrow::Status::Invalid(
                            "Non-nullable column contains null value: " +
                            request.name);
                    }
                    null_mask.push_back(is_null);
                    append_or_default(out, is_null, values->Value(i));
                }
                break;
            }
            case ParquetValueType::UInt64: {
                auto values =
                    std::static_pointer_cast<arrow::UInt64Array>(chunk);
                auto &out = std::get<std::vector<uint64_t>>(storage);
                for (int64_t i = 0; i < values->length(); ++i) {
                    bool is_null = values->IsNull(i);
                    if (is_null && !request.nullable) {
                        return arrow::Status::Invalid(
                            "Non-nullable column contains null value: " +
                            request.name);
                    }
                    null_mask.push_back(is_null);
                    append_or_default(out, is_null, values->Value(i));
                }
                break;
            }
            case ParquetValueType::Float32: {
                auto values =
                    std::static_pointer_cast<arrow::FloatArray>(chunk);
                auto &out = std::get<std::vector<float>>(storage);
                for (int64_t i = 0; i < values->length(); ++i) {
                    bool is_null = values->IsNull(i);
                    if (is_null && !request.nullable) {
                        return arrow::Status::Invalid(
                            "Non-nullable column contains null value: " +
                            request.name);
                    }
                    null_mask.push_back(is_null);
                    append_or_default(out, is_null, values->Value(i));
                }
                break;
            }
            case ParquetValueType::Float64: {
                auto values =
                    std::static_pointer_cast<arrow::DoubleArray>(chunk);
                auto &out = std::get<std::vector<double>>(storage);
                for (int64_t i = 0; i < values->length(); ++i) {
                    bool is_null = values->IsNull(i);
                    if (is_null && !request.nullable) {
                        return arrow::Status::Invalid(
                            "Non-nullable column contains null value: " +
                            request.name);
                    }
                    null_mask.push_back(is_null);
                    append_or_default(out, is_null, values->Value(i));
                }
                break;
            }
            case ParquetValueType::Bool: {
                auto values =
                    std::static_pointer_cast<arrow::BooleanArray>(chunk);
                auto &out = std::get<std::vector<bool>>(storage);
                for (int64_t i = 0; i < values->length(); ++i) {
                    bool is_null = values->IsNull(i);
                    if (is_null && !request.nullable) {
                        return arrow::Status::Invalid(
                            "Non-nullable column contains null value: " +
                            request.name);
                    }
                    null_mask.push_back(is_null);
                    out.push_back(!is_null && values->Value(i));
                }
                break;
            }
            case ParquetValueType::Utf8: {
                auto &out = std::get<std::vector<std::string>>(storage);
                if (chunk->type_id() == arrow::Type::STRING) {
                    auto values =
                        std::static_pointer_cast<arrow::StringArray>(chunk);
                    for (int64_t i = 0; i < values->length(); ++i) {
                        bool is_null = values->IsNull(i);
                        if (is_null && !request.nullable) {
                            return arrow::Status::Invalid(
                                "Non-nullable column contains null value: " +
                                request.name);
                        }
                        null_mask.push_back(is_null);
                        append_or_default(out, is_null,
                                          std::string(values->GetView(i)));
                    }
                } else {
                    auto values =
                        std::static_pointer_cast<arrow::LargeStringArray>(
                            chunk);
                    for (int64_t i = 0; i < values->length(); ++i) {
                        bool is_null = values->IsNull(i);
                        if (is_null && !request.nullable) {
                            return arrow::Status::Invalid(
                                "Non-nullable column contains null value: " +
                                request.name);
                        }
                        null_mask.push_back(is_null);
                        append_or_default(out, is_null,
                                          std::string(values->GetView(i)));
                    }
                }
                break;
            }
            case ParquetValueType::Binary: {
                auto &out =
                    std::get<std::vector<std::vector<uint8_t>>>(storage);
                if (chunk->type_id() == arrow::Type::BINARY) {
                    auto values =
                        std::static_pointer_cast<arrow::BinaryArray>(chunk);
                    for (int64_t i = 0; i < values->length(); ++i) {
                        bool is_null = values->IsNull(i);
                        if (is_null && !request.nullable) {
                            return arrow::Status::Invalid(
                                "Non-nullable column contains null value: " +
                                request.name);
                        }
                        null_mask.push_back(is_null);
                        if (is_null) {
                            out.emplace_back();
                        } else {
                            auto view = values->GetView(i);
                            out.emplace_back(
                                reinterpret_cast<const uint8_t *>(view.data()),
                                reinterpret_cast<const uint8_t *>(view.data()) +
                                    view.size());
                        }
                    }
                } else {
                    auto values =
                        std::static_pointer_cast<arrow::LargeBinaryArray>(
                            chunk);
                    for (int64_t i = 0; i < values->length(); ++i) {
                        bool is_null = values->IsNull(i);
                        if (is_null && !request.nullable) {
                            return arrow::Status::Invalid(
                                "Non-nullable column contains null value: " +
                                request.name);
                        }
                        null_mask.push_back(is_null);
                        if (is_null) {
                            out.emplace_back();
                        } else {
                            auto view = values->GetView(i);
                            out.emplace_back(
                                reinterpret_cast<const uint8_t *>(view.data()),
                                reinterpret_cast<const uint8_t *>(view.data()) +
                                    view.size());
                        }
                    }
                }
                break;
            }
            }
        }

        if (null_mask.size() != output.row_count) {
            return arrow::Status::Invalid(
                "Unexpected extracted row count for column: " + request.name);
        }
    }

    return arrow::Status::OK();
}
