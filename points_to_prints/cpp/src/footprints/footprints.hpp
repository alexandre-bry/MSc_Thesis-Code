#pragma once

#include <string>

#include <arrow/api.h>

struct RoofprintsAndLod22ToFootprintsOptions {
    std::string input_roofprints_file;
    std::string input_lod22_file;
    std::string input_points_file;
    std::string output_footprints_file;
    std::string output_points_file;
    bool overwrite;
};

arrow::Status roofprints_and_lod22_to_footprints(
    const std::string &input_roofprints_file,
    const std::string &input_lod22_file, const std::string &input_points_file,
    const std::string &output_footprints_file,
    const std::string &output_points_file, bool overwrite);