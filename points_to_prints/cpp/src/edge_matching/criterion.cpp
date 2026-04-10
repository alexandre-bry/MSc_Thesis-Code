#include "criterion.hpp"

#include "../utils/cgal.hpp"
#include "constants.hpp"
#include <CGAL/enum.h>

double LinearCriterion::evaluate_segments(
    const std::vector<Segment_2> &segments,
    const std::vector<double> &segments_initial_length) const {
    // Check that the input vectors have the same size
    if (segments.size() != segments_initial_length.size()) {
        throw std::invalid_argument(
            "Segments and segments_initial_length must have the same size");
    }

    // Compute the perimeters
    double current_perimeter = 1e-6;
    double initial_perimeter = 1e-6;
    for (size_t i = 0; i < segments.size(); ++i) {
        current_perimeter += std::sqrt(segments.at(i).squared_length());
        initial_perimeter += segments_initial_length.at(i);
    }

    // Compute the proximity value
    std::vector<double> point_best_score(points.size(), 0.0);
    double proximity_value = 0.0;
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto &segment = segments.at(i);

        UnitVector_2 segment_normal(
            segment.supporting_line().to_vector().perpendicular(
                CGAL::CLOCKWISE));

        // std::cout << "Evaluating segment " << i << ": " << segment <<
        // std::endl;

        // Query the points that may be close enough
        std::vector<std::size_t> nearby_point_indices =
            las_kd_tree.search_indices_in_box(segment.bbox(),
                                              EDGE_CRITERION_MAX_DISTANCE);

        // std::cout << "Number of nearby points: " <<
        // nearby_point_indices.size()
        //           << std::endl;

        // int num_points_not_on_segment = 0;
        // int num_points_far_from_segment = 0;
        // int num_points_considered = 0;

        for (std::size_t point_index : nearby_point_indices) {
            const Point_2 &point = points.at(point_index);
            double weight = weights.at(point_index);

            // Check if the projection of the point on the segment is within the
            // segment
            const Point_2 &closest_point =
                segment.supporting_line().projection(point);
            auto dist_proj_segment =
                CGAL::squared_distance(closest_point, segment);
            if (dist_proj_segment > 1e-6) {
                // num_points_not_on_segment++;
                continue;
            }

            // Check if the distance from the point to the segment smaller than
            // the threshold
            double distance = std::sqrt(CGAL::squared_distance(point, segment));
            if (distance > EDGE_CRITERION_MAX_DISTANCE) {
                // num_points_far_from_segment++;
                continue;
            }

            // Check the angle between the point normal and the segment normal
            UnitVector_2 point_normal = point_normals.at(point_index);
            double dot_product = point_normal * segment_normal;
            if (dot_product < 0.5) {
                continue;
            }
            weight *= dot_product;

            // Compute the proximity score
            double proximity_score =
                weight * (1.0 - distance / EDGE_CRITERION_MAX_DISTANCE);
            point_best_score[point_index] =
                std::max(point_best_score[point_index], proximity_score);

            // num_points_considered++;
        }
        // std::cout << "Not on segment: " << num_points_not_on_segment
        //           << ", Far from segment: " << num_points_far_from_segment
        //           << ", Considered: " << num_points_considered << std::endl;
    }
    proximity_value =
        -std::accumulate(point_best_score.begin(), point_best_score.end(), 0.0);
    proximity_value *= EDGE_CRITERION_POINT_DENSITY;

    // // Compute the perimeter ratio value
    // double perimeter_ratio_value;
    // if (current_perimeter > initial_perimeter) {
    //     perimeter_ratio_value =
    //         (current_perimeter / initial_perimeter) * initial_perimeter;
    // } else {
    //     perimeter_ratio_value =
    //         (initial_perimeter / current_perimeter) * initial_perimeter;
    // }
    // perimeter_ratio_value *= this->alpha_ratio;

    // Compute the mean edge length ratio value
    double mean_edge_length_ratio_value = 0.0;
    for (size_t i = 0; i < segments.size(); ++i) {
        double current_length = std::sqrt(segments.at(i).squared_length());
        double initial_length = segments_initial_length.at(i);
        if (current_length > initial_length) {
            mean_edge_length_ratio_value += (current_length / initial_length);
        } else {
            mean_edge_length_ratio_value += (initial_length / current_length);
        }
    }
    mean_edge_length_ratio_value /= segments.size();
    mean_edge_length_ratio_value *= this->alpha_ratio;

    // Compute the perimeter absolute value
    double perimeter_abs_value = this->alpha_absolute * current_perimeter;

    return proximity_value + mean_edge_length_ratio_value + perimeter_abs_value;
}