#include "criterion.hpp"

#include "../points.hpp"
#include "constants.hpp"

double LinearCriterion::evaluate_segments(
    const std::vector<Segment_2> &segments,
    const std::vector<double> &segments_initial_length) const {
    // Compute the proximity value
    double proximity_value = 0.0;
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto &segment = segments[i];
        double initial_length = segments_initial_length[i];

        // Query the points that may be close enough
        std::vector<std::size_t> nearby_point_indices =
            las_kd_tree.search_indices_in_box(segment.bbox(),
                                              EDGE_CRITERION_MAX_DISTANCE);

        for (std::size_t point_index : nearby_point_indices) {
            const Point_2 &point = points[point_index];
            double weight = weights[point_index];

            // Check if the projection of the point on the segment is within the
            // segment
            const Point_2 &closest_point =
                segment.supporting_line().projection(point);
            if (!segment.has_on(closest_point)) {
                continue;
            }

            // Check if the distance from the point to the segment smaller than
            // the threshold
            double distance = std::sqrt(CGAL::squared_distance(point, segment));
            if (distance > EDGE_CRITERION_MAX_DISTANCE) {
                continue;
            }

            // Compute the proximity score
            double proximity_score =
                weight * (1.0 - distance / EDGE_CRITERION_MAX_DISTANCE);
            proximity_value -= proximity_score;
        }
    }

    // Compute the perimeter ratio value
    double current_perimeter = 1e-6;
    double initial_perimeter = 1e-6;
    for (size_t i = 0; i < segments.size(); ++i) {
        current_perimeter += std::sqrt(segments[i].squared_length());
        initial_perimeter += segments_initial_length[i];
    }
    double perimeter_ratio_value;
    if (current_perimeter > initial_perimeter) {
        perimeter_ratio_value =
            this->alpha_ratio * (current_perimeter / initial_perimeter);
    } else {
        perimeter_ratio_value =
            this->alpha_ratio * (initial_perimeter / current_perimeter);
    }

    // Compute the perimeter absolute value
    double perimeter_abs_value = this->alpha_absolute * current_perimeter;

    return proximity_value + perimeter_ratio_value + perimeter_abs_value;
}