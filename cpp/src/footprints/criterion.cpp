#include "criterion.hpp"

#include "../geom/cgal.hpp"
#include "constants.hpp"
#include <CGAL/enum.h>

double CriterionFootprints::evaluate_segments(
    const std::vector<Segment_2> &segments,
    const std::vector<double> &segments_initial_length,
    const std::vector<UnitVector_2> &segments_inward_normals) const {
    // Check that the input vectors have the same size
    if (segments.size() != segments_initial_length.size()) {
        throw std::invalid_argument(
            "Segments and segments_initial_length must have the same size");
    }

    // std::cout << "Evaluating criterion for " << segments.size()
    //           << " segments with " << points.size() << " points" <<
    //           std::endl;

    // Compute the proximity value
    std::vector<double> point_energy(points.size(), 0.0);
    std::vector<double> point_closest_segment_distance(
        points.size(), std::numeric_limits<double>::max());
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto &segment = segments.at(i);

        // UnitVector_2 segment_inward_normal(
        //     segment.supporting_line().to_vector().perpendicular(
        //         CGAL::COUNTERCLOCKWISE));
        UnitVector_2 segment_inward_normal = segments_inward_normals.at(i);

        // Query the points that may be close enough
        std::vector<std::size_t> nearby_point_indices;
        las_kd_tree.search_indices_in_box(
            segment.bbox(), distance_close + distance_penalty + 5.0,
            nearby_point_indices);

        for (std::size_t point_index : nearby_point_indices) {
            const Point_2 &point = points.at(point_index);
            double weight = weights.at(point_index);
            LASclassification::Value point_class =
                point_classes.at(point_index);

            // Skip vegetation points
            if (point_class == LASclassification::Value::LowVegetation ||
                point_class == LASclassification::Value::MediumVegetation ||
                point_class == LASclassification::Value::HighVegetation) {
                continue;
            }

            // Check if the projection of the point on the segment is within the
            // segment
            const Point_2 &point_proj_line =
                segment.supporting_line().projection(point);
            auto dist_proj_segment =
                CGAL::squared_distance(point_proj_line, segment);
            if (dist_proj_segment > 1e-6) {
                continue;
            }

            // Compute the signed distance, positive if the point is in the
            // direction of the inward normal
            Vector_2 segment_to_point = point - point_proj_line;
            double signed_distance = segment_to_point * segment_inward_normal;
            double absolute_distance = std::abs(signed_distance);

            // Continue if the point is closer to another segment
            if (absolute_distance >=
                point_closest_segment_distance.at(point_index)) {
                continue;
            }
            point_closest_segment_distance.at(point_index) = absolute_distance;

            bool ground_point =
                (point_class == LASclassification::Value::Ground);
            double energy = 0.0;
            if (signed_distance < -distance_close || signed_distance > distance_close + distance_penalty) {
                continue;
            } else if (signed_distance < distance_close) {
                energy = -weight * (1.0 - absolute_distance / distance_close);
                if (ground_point) {
                    energy *= 0.3;
                } else {
                    energy *= 1.0;
                }
            } else {
                energy = weight * std::min((signed_distance - distance_close) /
                                               distance_penalty,
                                           1.0);
                if (ground_point) {
                    energy *= 1.0;
                } else {
                    energy *= 0.3;
                }
            }

            point_energy.at(point_index) = energy;
        }
    }

    double proximity_value =
        std::accumulate(point_energy.begin(), point_energy.end(), 0.0);

    double total_value = proximity_value;

    return total_value;
}