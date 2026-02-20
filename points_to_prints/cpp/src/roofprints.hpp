#pragma once

#include <string>
#include <vector>

#include "geometry.hpp"
#include "las.hpp"
#include "points.hpp"

struct OutlineWithPoints {
    PolygonZWithAttributes outline;
    std::vector<Point3DWithAttributes> points_in_buffer;

    OutlineWithPoints(PolygonZWithAttributes outline_,
                      std::vector<Point3DWithAttributes> &points_in_buffer_)
        : outline(std::move(outline_)),
          points_in_buffer(std::move(points_in_buffer_)) {}
};

void select_outlines_in_las(
    CustomLasReader &reader,
    const std::vector<MultiPolygonZWithAttributes> &outlines,
    double buffer_distance,
    std::vector<MultiPolygonZWithAttributes> &selected_outlines);

void select_points_for_outline(
    CustomLasReader &las_reader,
    const std::vector<MultiPolygonZWithAttributes> &outlines,
    double buffer_distance,
    std::vector<OutlineWithPoints> &outlines_with_points);

void compute_roofprints_in_las(
    CustomLasReader &las_reader,
    const std::vector<MultiPolygonZWithAttributes> &all_outlines,
    std::vector<PolygonZWithAttributes> &roofprints, double las_buffer_distance,
    double outline_buffer_distance);

struct ComputeRoofprintsOptions {
    std::string input_las_file;
    std::string input_bd_topo_file;
    std::string output_roofprints_file;
    double las_buffer_distance;
    double outline_buffer_distance;
    bool overwrite;
};

void compute_roofprints(const std::string &input_las_file,
                        const std::string &input_bd_topo_file,
                        const std::string &output_roofprints_file,
                        double las_buffer_distance,
                        double outline_buffer_distance, bool overwrite);