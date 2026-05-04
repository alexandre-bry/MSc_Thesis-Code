#pragma once

#include "../utils/cgal.hpp"

void score_line_translations(const Line_2 &line,
                             const std::vector<Point_3> &points,
                             const UnitVector_2 &translation_direction,
                             const std::vector<double> &translations,
                             double distance_threshold, double distance_penalty,
                             std::vector<double> &scores);