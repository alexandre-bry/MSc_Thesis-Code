#pragma once

#include "../utils/cgal.hpp"
#include "../utils/kd_tree.hpp"
#include "constants.hpp"

class LinearCriterion {
  private:
    std::vector<Point_2> points;
    std::vector<double> weights;
    std::vector<Vector_2> point_inward_dirs;
    double max_distance;
    double alpha_mean_edges_ratio;
    double alpha_mean_edges_difference;
    double alpha_perimeter_absolute;
    KdTree_2 las_kd_tree;

  public:
    LinearCriterion(
        std::vector<Point_2> points, std::vector<double> weights,
        std::vector<Vector_2> point_inward_dirs,
        double max_distance = EDGE_CRITERION_MAX_DISTANCE,
        double alpha_mean_edges_ratio = EDGE_CRITERION_ALPHA_MEAN_EDGES_RATIO,
        double alpha_mean_edges_difference =
            EDGE_CRITERION_ALPHA_MEAN_EDGES_DIFFERENCE,
        double alpha_perimeter_absolute =
            EDGE_CRITERION_ALPHA_PERIMETER_ABSOLUTE)
        : points(points), weights(weights),
          point_inward_dirs(point_inward_dirs), max_distance(max_distance),
          alpha_mean_edges_ratio(alpha_mean_edges_ratio),
          alpha_mean_edges_difference(alpha_mean_edges_difference),
          alpha_perimeter_absolute(alpha_perimeter_absolute),
          las_kd_tree(points) {}

    double evaluate_segments(
        const std::vector<Segment_2> &segments,
        const std::vector<double> &segments_initial_length,
        const std::vector<UnitVector_2> &segments_inward_normals) const;
};