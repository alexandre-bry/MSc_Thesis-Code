#pragma once

#include "../utils/cgal.hpp"
#include "../utils/kd_tree.hpp"
#include "constants.hpp"

class LinearCriterion {
  private:
    std::vector<Point_2> points;
    std::vector<double> weights;
    std::vector<Vector_2> point_inner_dirs;
    double max_distance;
    double alpha_ratio;
    double alpha_absolute;
    KdTree_2 las_kd_tree;

  public:
    LinearCriterion(std::vector<Point_2> points, std::vector<double> weights,
                    std::vector<Vector_2> point_inner_dirs,
                    double max_distance = EDGE_CRITERION_MAX_DISTANCE,
                    double alpha_ratio = EDGE_CRITERION_ALPHA_RATIO,
                    double alpha_absolute = EDGE_CRITERION_ALPHA_ABSOLUTE)
        : points(points), weights(weights), point_inner_dirs(point_inner_dirs),
          max_distance(max_distance), alpha_ratio(alpha_ratio),
          alpha_absolute(alpha_absolute), las_kd_tree(points) {}

    double evaluate_segments(
        const std::vector<Segment_2> &segments,
        const std::vector<double> &segments_initial_length,
        const std::vector<UnitVector_2> &segments_inner_normals) const;
};