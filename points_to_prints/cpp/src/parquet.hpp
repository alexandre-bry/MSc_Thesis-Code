#include <string>

#include "arrow/status.h"

struct TestParquetOptions {
    std::string input_file;
    bool overwrite;
};

arrow::Status open_parquet(std::string &input_file);