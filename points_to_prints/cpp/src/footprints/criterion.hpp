#pragma once

#include "../edge_matching/criterion.hpp"
#include "../geom/cgal.hpp"
#include "../geom/kd_tree.hpp"
#include "../las/enums.hpp"
#include "constants.hpp"

class CriterionFootprints : public ICriterion {
  private:
    std::vector<Point_2> points;
    std::vector<double> weights;
    std::vector<LASclassification::Value> point_classes;
    double distance_close;
    double distance_penalty;
    KdTree_2 las_kd_tree;

  public:
    CriterionFootprints(
        std::vector<Point_2> points, std::vector<double> weights,
        std::vector<LASclassification::Value> point_classes,
        double distance_close = EDGE_CRITERION_DISTANCE_CLOSE,
        double distance_penalty = EDGE_CRITERION_DISTANCE_PENALTY)
        : points(points), weights(weights), point_classes(point_classes),
          distance_close(distance_close), distance_penalty(distance_penalty),
          las_kd_tree(points) {}

    double evaluate_segments(const std::vector<Segment_2> &segments,
                             const std::vector<double> &segments_initial_length,
                             const std::vector<UnitVector_2>
                                 &segments_inward_normals) const override;
};