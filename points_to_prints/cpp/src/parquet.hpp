#pragma once

#include <string>

#include "arrow/status.h"

#include <ogr_api.h>
#include <ogr_geometry.h>

#include "geometry.hpp"

struct TestParquetOptions {
    std::string input_file;
};

arrow::Status open_parquet(std::string &input_file);

struct ReadWriteBDTOPOOptions {
    std::string input_parquet_file;
    std::string output_parquet_file;
    bool overwrite;
};

arrow::Status read_building_outlines_from_bd_topo(
    const std::string &bd_topo_parquet_file,
    std::vector<MultiPolygonZWithAttributes> &outlines);

arrow::Status write_multi_polygons_to_parquet(
    const std::vector<MultiPolygonZWithAttributes> &multi_polygons,
    const std::string &output_file, bool overwrite);

arrow::Status read_write_bd_topo(const std::string &input_parquet_file,
                                 const std::string &output_parquet_file,
                                 bool overwrite);