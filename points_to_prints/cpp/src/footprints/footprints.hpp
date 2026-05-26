#pragma once

#include <string>

#include <arrow/api.h>

struct Roofprints3DToFootprintsOptions {
    std::string input_roofprints_file;
    std::string points_file;
    std::string output_footprints_file;
    std::string output_points_file;
    bool overwrite;
};

arrow::Status roofprints_3d_to_footprints(
    const std::string &input_roofprints_file, const std::string &points_file,
    const std::string &output_footprints_file,
    const std::string &output_points_file, bool overwrite);