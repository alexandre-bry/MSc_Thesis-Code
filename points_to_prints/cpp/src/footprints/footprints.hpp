#pragma once

#include <string>

#include <arrow/api.h>

struct RoofsToFootprintsOptions {
    std::string input_roofs_file;
    std::string input_points_file;
    std::string output_footprints_file;
    std::string output_points_file;
    bool overwrite;
};

arrow::Status roofs_to_footprints(const std::string &input_roofs_file,
                                  const std::string &input_points_file,
                                  const std::string &output_footprints_file,
                                  const std::string &output_points_file,
                                  bool overwrite);