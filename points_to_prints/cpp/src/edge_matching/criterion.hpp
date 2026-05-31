#pragma once

#include "../utils/cgal.hpp"

class ICriterion {
  public:
    virtual ~ICriterion() = default;

    virtual double evaluate_segments(
        const std::vector<Segment_2> &segments,
        const std::vector<double> &segments_initial_length,
        const std::vector<UnitVector_2> &segments_inward_normals) const {
        throw std::logic_error(
            "ICriterion::evaluate_segments must be overridden in subclasses");
    };
};
