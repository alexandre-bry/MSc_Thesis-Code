#include "roofprints.hpp"

#include <cstddef>
#include <filesystem>
#include <iostream>
#include <memory>
#include <random>
#include <type_traits>
#include <vector>

#include <ogr_geometry.h>

#include "geometry.hpp"
#include "kd_tree.hpp"
#include "las/reader.hpp"
#include "parquet.hpp"
#include "pbar.hpp"
#include "pca.hpp"
#include "points.hpp"
#include "ransac.hpp"
#include "utils/cgal.hpp"

void select_outlines_in_las(
    NewLasReader &reader,
    const std::vector<MultiPolygonZWithAttributes> &outlines,
    double buffer_distance,
    std::vector<MultiPolygonZWithAttributes> &selected_outlines) {
    selected_outlines.clear();

    OGREnvelopePtr las_bounding_box = reader.points->bounding_box();
    las_bounding_box->MinX += buffer_distance;
    las_bounding_box->MinY += buffer_distance;
    las_bounding_box->MaxX -= buffer_distance;
    las_bounding_box->MaxY -= buffer_distance;

    for (const auto &outline : outlines) {
        OGREnvelopePtr outline_bounding_box = outline.bounding_box();

        if (las_bounding_box->Contains(*outline_bounding_box.get())) {
            selected_outlines.push_back(outline);
        }
    }
}

void select_points_per_outlines(
    NewLasReader &las_reader,
    const std::vector<MultiPolygonZWithAttributes> &outlines,
    double buffer_distance,
    std::vector<OutlineWithPoints> &outlines_with_points) {
    outlines_with_points.clear();

    OGREnvelopePtr las_bounding_box = las_reader.points->bounding_box();

    // Build a KD-tree for efficient spatial queries on the LAS points
    std::cout << "Building KD-tree for LAS points..." << std::endl;
    const PtsStructs::StoragePtr points = las_reader.points;
    std::cout << "Number of points: " << points->point_count() << std::endl;
    std::vector<Point_2> cgal_points;
    cgal_points.reserve(points->point_count());
    for (PtsStructs::PointId i(0); i < points->point_count(); ++i) {
        const auto &point = points->get_point(i);
        cgal_points.emplace_back(point.x(), point.y());
    }
    KdTree_2 kd_tree(cgal_points);

    // For each outline, find the points that are within its bounding box plus a
    // buffer
    ProgressBarTotal bar(outlines.size(), "Selecting points for outlines");
    bar.start();
    for (const auto &outline : outlines) {
        for (const auto &polygon : outline.get_polygons_with_attributes()) {
            OGREnvelopePtr bbox = polygon.bounding_box();
            bbox->MinX -= buffer_distance;
            bbox->MinY -= buffer_distance;
            bbox->MaxX += buffer_distance;
            bbox->MaxY += buffer_distance;

            std::vector<PtsStructs::PointId> points_on_outline;
            std::vector<std::size_t> indices_in_area =
                kd_tree.search_indices_in_box({bbox->MinX, bbox->MinY},
                                              {bbox->MaxX, bbox->MaxY});
            for (auto idx : indices_in_area) {
                points_on_outline.push_back(PtsStructs::PointId(idx));
            }
            outlines_with_points.emplace_back(polygon, points_on_outline);
        }
        bar.increment(1);
    }
    bar.finish();
}

const double METRIC_INTERVAL = 0.3;
const double EDGE_OPTIMIZATION_INTERVAL = 0.5;
const double PROJECTION_CHECK_MARGIN = 1.0;

double compute_metric(const std::vector<double> &offsets,
                      const std::vector<double> &weights,
                      double chosen_offset) {
    double total_score = 0.0;
    for (std::size_t i = 0; i < offsets.size(); ++i) {
        double dist = std::abs(offsets[i] - chosen_offset);
        double score =
            (dist < METRIC_INTERVAL) ? (METRIC_INTERVAL - dist) : 0.0;
        total_score += weights[i] * std::sqrt(score);
    }
    return total_score;
}

// double best_proximity_offset(const std::vector<Point_2> &points,
//                              const Point_2 &base,
//                              const Vector_2 &perp_dir, // unit vector
//                              double max_offset, double epsilon = 1e-6) {
//     std::vector<double> offsets;

//     for (const auto &p : points) {
//         offsets.push_back(perp_dir * (p - base));
//     }

//     // Ternary search on t in [-max_offset, max_offset]
//     double left = -max_offset, right = max_offset;
//     while (right - left > epsilon) {
//         double m1 = left + (right - left) / 3;
//         double m2 = right - (right - left) / 3;

//         double f1 = compute_metric(offsets, m1);
//         double f2 = compute_metric(offsets, m2);

//         if (f1 > f2) {
//             right = m2;
//         } else {
//             left = m1;
//         }
//     }

//     return (left + right) / 2;
// }

// struct EdgeCriterionConstant3D {
//     const std::vector<Point_3> &points;

//     EdgeCriterionConstant3D(const std::vector<Point_3> &points)
//         : points(points) {}

//     double get_score(const Segment_3 &segment) const {
//         double total_score = 0.0;
//         for (const auto &point : points) {
//             double dist = std::sqrt(CGAL::squared_distance(segment, point));
//             double score = (dist < METRIC_INTERVAL) ? 1.0 : 0.0;
//             total_score += score;
//         }
//         return total_score;
//     }

//     double get_score(const Line_3 &line) const {
//         double total_score = 0.0;
//         for (const auto &point : points) {
//             double dist = std::sqrt(CGAL::squared_distance(line, point));
//             double score = (dist < METRIC_INTERVAL) ? 1.0 : 0.0;
//             total_score += score;
//         }
//         return total_score;
//     }
// };

// struct EdgeCriterionLinear3D {
//     const std::vector<Point_3> &points;

//     EdgeCriterionLinear3D(const std::vector<Point_3> &points)
//         : points(points) {}

//     double get_score(const Segment_3 &segment) const {
//         double total_score = 0.0;
//         for (const auto &point : points) {
//             double dist = std::sqrt(CGAL::squared_distance(segment, point));
//             double score =
//                 (dist < METRIC_INTERVAL) ? (METRIC_INTERVAL - dist) : 0.0;
//             total_score += score;
//         }
//         return total_score;
//     }

//     double get_score(const Line_3 &line) const {
//         double total_score = 0.0;
//         for (const auto &point : points) {
//             double dist = std::sqrt(CGAL::squared_distance(line, point));
//             double score =
//                 (dist < METRIC_INTERVAL) ? (METRIC_INTERVAL - dist) : 0.0;
//             total_score += score;
//         }
//         return total_score;
//     }
// };

// const double MIN_SCORE_THRESHOLD = 5.0;

// struct RANSAC3D {
//   private:
//     const std::vector<Point_3> &points;
//     int max_iterations;
//     double distance_threshold;
//     std::random_device rd;
//     std::mt19937 gen;
//     std::uniform_int_distribution<std::size_t> dis;
//     EdgeCriterionConstant3D edge_criterion_constant;
//     EdgeCriterionLinear3D edge_criterion_linear;

//   public:
//     RANSAC3D(const std::vector<Point_3> &points, int max_iterations,
//              double distance_threshold)
//         : points(points), max_iterations(max_iterations),
//           distance_threshold(distance_threshold), dis(0, points.size() - 1),
//           edge_criterion_constant(points), edge_criterion_linear(points) {
//         gen.seed(rd());
//     }

//     std::pair<std::unique_ptr<Point_3>, std::unique_ptr<Point_3>>
//     get_random_pair() {
//         std::size_t idx1 = dis(gen);
//         std::size_t idx2 = dis(gen);
//         while (idx2 == idx1) {
//             idx2 = dis(gen);
//         }
//         return {std::make_unique<Point_3>(points[idx1]),
//                 std::make_unique<Point_3>(points[idx2])};
//     }

//     std::vector<Line_3> run() {
//         std::vector<Line_3> best_lines;

//         for (int i = 0; i < max_iterations; ++i) {
//             auto [p1, p2] = get_random_pair();
//             Line_3 line(*p1, *p2);
//             double score = edge_criterion_linear.get_score(line);
//             if (score > MIN_SCORE_THRESHOLD) {
//                 best_lines.clear();
//                 best_lines.push_back(line);
//             }
//         }

//         return best_lines;
//     }
// };

Line_2 fit_line(const std::vector<Point_2> &points) {
    auto [normal, line, eigenvalues] = compute_pca_once(points);
    return line;
}

std::tuple<double, Segment_2> best_proximity_offset(
    const std::vector<Point_2> &points, const std::vector<double> &weights,
    const Point_2 &base_start, const Point_2 &base_end,
    const Vector_2 &perp_dir, double max_offset, double offset_step) {
    double best_offset = 0.0;
    double best_score = 0.0;

    Vector_2 start_to_end = base_end - base_start;
    double start_to_end_length = CGAL::sqrt(start_to_end.squared_length());
    Vector_2 start_to_end_unit = start_to_end / start_to_end_length;

    // Compute the offsets of the points from the base line
    std::vector<double> offsets;
    for (const auto &p : points) {
        // Check if the point projected on the edge is within the edge
        // segment
        Vector_2 start_to_p = p - base_start;
        double projection_length = start_to_p * start_to_end_unit;
        if (projection_length < -PROJECTION_CHECK_MARGIN ||
            projection_length > start_to_end_length + PROJECTION_CHECK_MARGIN) {
            continue;
        }

        double perp_distance = perp_dir * (p - base_start);
        offsets.push_back(perp_distance);
    }

    // Find the offset that maximizes the metric using a linear search
    for (double offset = -max_offset; offset <= max_offset;
         offset += offset_step) {
        double score = compute_metric(offsets, weights, offset);
        if (score > best_score) {
            best_score = score;
            best_offset = offset;
        }
    }

    // Get the new offseted edge defined by the best offset
    Point_2 new_start = base_start + perp_dir * best_offset;
    Point_2 new_end = base_end + perp_dir * best_offset;
    Segment_2 new_segment(new_start, new_end);

    // // Identify the points that are close to the new edge defined by the best
    // // offset and fit a line if there are enough points
    // std::vector<Point_2> points_for_fitting;
    // for (const auto &p : points) {
    //     double perp_distance = perp_dir * (p - new_start);
    //     if (std::abs(perp_distance) < EDGE_OPTIMIZATION_INTERVAL) {
    //         points_for_fitting.push_back(p);
    //     }
    // }

    // if (points_for_fitting.size() < 2) {
    //     // Not enough points to fit a line, return the original line with the
    //     // best offset
    //     return {best_offset, Segment_2(new_start, new_end)};
    // }

    // Line_2 fitted_line = fit_line(points_for_fitting);
    // Segment_2 fitted_segment = Segment_2(fitted_line.projection(new_start),
    //                                      fitted_line.projection(new_end));

    return {best_offset, new_segment};
}

// double best_proximity_offset(const std::vector<Point_2> &points,
//                              const std::vector<double> &weights,
//                              const std::vector<Segment_2> &base_edges,
//                              const Vector_2 &perp_dir, double max_offset,
//                              double offset_step) {
//     double best_offset = 0.0;
//     double best_score = 0.0;

//     Vector_2 start_to_end = base_end - base_start;
//     double start_to_end_length = CGAL::sqrt(start_to_end.squared_length());
//     Vector_2 start_to_end_unit = start_to_end / start_to_end_length;

//     std::vector<double> offsets;
//     for (const auto &p : points) {
//         // Check if the point projected on the edge is within the edge
//         // segment
//         Vector_2 start_to_p = p - base_start;
//         double projection_length = start_to_p * start_to_end_unit;
//         if (projection_length < -PROJECTION_CHECK_MARGIN ||
//             projection_length > start_to_end_length +
//             PROJECTION_CHECK_MARGIN) { continue;
//         }

//         double perp_distance = perp_dir * (p - base_start);
//         offsets.push_back(perp_distance);
//     }

//     for (double offset = -max_offset; offset <= max_offset;
//          offset += offset_step) {
//         double score = compute_metric(offsets, weights, offset);
//         if (score > best_score) {
//             best_score = score;
//             best_offset = offset;
//         }
//     }

//     return best_offset;
// }

/**
 * @brief Get the angle in degrees between two edges defined by their start and
 * end points. The angle is always between 0 and 90 degrees, as it takes the
 * smaller angle between the edges.
 *
 * @param edge1
 * @param edge2
 * @return CustomCGAL::Angle
 */
CustomCGAL::Angle get_angle_between(std::pair<Point_2, Point_2> edge1,
                                    std::pair<Point_2, Point_2> edge2) {
    Vector_2 dir1 = edge1.second - edge1.first;
    Vector_2 dir2 = edge2.second - edge2.first;
    double angle = CustomCGAL::angle(dir1, dir2).in_degrees();
    // Ensure the angle is in the range [0, 180]
    if (angle > 180.0) {
        angle = 360.0 - angle;
    }
    // Ensure to take the smaller angle between the edges
    if (angle > 90.0) {
        angle = 180.0 - angle;
    }
    return CustomCGAL::Angle::from_degrees(angle);
}

/**
 * @brief Get the angle in degrees between two edges defined by their start and
 * end points. The angle is always between 0 and 90 degrees, as it takes the
 * smaller angle between the edges.
 *
 * @param edge1
 * @param edge2
 * @return CustomCGAL::Angle
 */
CustomCGAL::Angle get_angle_between(Segment_2 edge1, Segment_2 edge2) {
    Vector_2 dir1 = edge1.to_vector();
    Vector_2 dir2 = edge2.to_vector();
    double angle = CustomCGAL::angle(dir1, dir2).in_degrees();
    // Ensure the angle is in the range [0, 180]
    if (angle > 180.0) {
        angle = 360.0 - angle;
    }
    // Ensure to take the smaller angle between the edges
    if (angle > 90.0) {
        angle = 180.0 - angle;
    }
    return CustomCGAL::Angle::from_degrees(angle);
}

double MAX_OFFSET = 2.0;
double INTERVAL = 0.001;

const double ANGLE_THRESHOLD_DEGREES = 22.5;
const double MAX_PROJECTION_DISTANCE_FACTOR = 0.1;

struct PolygonHandler {
  private:
    std::vector<PtsStructs::PointId> point_ids;
    std::vector<bool> is_minor; // Whether the point connects two edges that
                                // should be processed together
    const PtsStructs::StoragePtr points;

  public:
    PolygonHandler(const std::vector<PtsStructs::PointId> &point_ids_,
                   const PtsStructs::StoragePtr points_)
        : point_ids(point_ids_), is_minor(point_ids_.size(), false),
          points(points_) {

        for (size_t i = 0; i < point_ids.size(); ++i) {
            const Point_2 &point = points->get_point_2d(point_ids[i]);
            const Point_2 &point_prev = points->get_point_2d(
                point_ids[(i + point_ids.size() - 1) % point_ids.size()]);
            const Point_2 &point_next =
                points->get_point_2d(point_ids[(i + 1) % point_ids.size()]);

            Segment_2 edge_prev(point_prev, point);
            Segment_2 edge_next(point, point_next);

            CustomCGAL::Angle angle = get_angle_between(edge_prev, edge_next);
            if (angle.in_degrees() < ANGLE_THRESHOLD_DEGREES) {
                // Check if the projection of the common point of the two edges
                // on the edge with the two other points is small
                Segment_2 edge_new(point_prev, point_next);
                double edge_new_length = std::sqrt(edge_new.squared_length());
                Line_2 line_new(point_prev, point_next);
                Point_2 projection = line_new.projection(point);
                double distance_to_projection =
                    std::sqrt(CGAL::squared_distance(projection, point));

                if (distance_to_projection <
                    MAX_PROJECTION_DISTANCE_FACTOR * edge_new_length) {
                    is_minor[i] = true;
                }
            }
        }
    }
};

PolygonZWithAttributes
compute_roofprint(const OutlineWithPoints &outline_with_points,
                  PtsStructs::StoragePtr points) {
    const auto &points_ids = outline_with_points.points_in_buffer;

    // Project the points in 2D in CGAL format
    std::vector<Point_2> points_2d(points_ids.size());
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::min();
    for (size_t i = 0; i < points_ids.size(); ++i) {
        const PtsStructs::PointId point_id = points_ids[i];
        const auto &point = points->get_point(point_id);
        points_2d[i] = Point_2(point.x(), point.y());
        min_z = std::min(min_z, point.z());
        max_z = std::max(max_z, point.z());
    }

    // Compute the weights for each point
    std::vector<double> weights(points_ids.size());
    for (size_t i = 0; i < points_ids.size(); ++i) {
        const PtsStructs::PointId point_id = points_ids[i];

        // Give more weight to higher points
        double z =
            points->get_field_as<double>(pdal::Dimension::Id::Z, point_id);
        double height_norm = (z - min_z + 1e-6) / (max_z - min_z + 1e-6);
        double height_factor = 1.0 + 2.0 * height_norm;

        // Give more weight to non-generated points
        uint8_t is_generated = points->get_field_as<uint8_t>(
            CustomDimensions::Id::IsGenerated, point_id);
        double generated_factor;
        if (is_generated == 0) {
            generated_factor = 1.0;
        } else if (is_generated == 1) {
            generated_factor = 0.5;
        } else if (is_generated == 2) {
            generated_factor = 0.1;
        } else {
            throw std::runtime_error("Unexpected value for IsGenerated field");
        }

        // Give more weight to points classified as building
        const auto cls_raw = points->get_field_as<
            std::underlying_type_t<LASclassification::Value>>(
            pdal::Dimension::Id::Classification, point_id);
        const auto cls = static_cast<LASclassification::Value>(cls_raw);
        double class_factor = 1.0;
        if (cls == LASclassification::Value::Building) {
            class_factor = 2.0;
        }

        // Combine the factors to get the final weight
        weights[i] = height_factor * generated_factor * class_factor;
    }

    // Find edges of the input outlines
    std::vector<std::pair<Point_2, Point_2>> initial_edges;
    const auto &polygon = outline_with_points.outline;
    const auto &exterior_ring = polygon.polygon->getExteriorRing();
    const auto num_points = exterior_ring->getNumPoints();
    for (int i = 0; i < num_points; ++i) {
        OGRPoint start, end;
        exterior_ring->getPoint(i, &start);
        exterior_ring->getPoint((i + 1) % num_points, &end);
        initial_edges.emplace_back(Point_2(start.getX(), start.getY()),
                                   Point_2(end.getX(), end.getY()));
    }

    // Simplify edges
    std::vector<std::pair<Point_2, Point_2>> edges;
    if (!initial_edges.empty()) {
        edges.push_back(initial_edges[0]);
        // Merge collinear edges
        for (size_t i = 1; i < initial_edges.size(); ++i) {
            const auto &prev_edge = edges.back();
            const auto &curr_edge = initial_edges[i];

            // // Get the angle between the two edges
            // CustomCGAL::Angle angle = get_angle_between(prev_edge,
            // curr_edge); if (std::abs(angle.in_degrees()) < 5.0) {
            //     // The edges are collinear, merge them by updating the
            //     end point edges.back().second = curr_edge.second;
            // } else {
            //     edges.push_back(curr_edge);
            // }

            CustomCGAL::Angle angle = get_angle_between(prev_edge, curr_edge);
            if (angle.in_degrees() < ANGLE_THRESHOLD_DEGREES) {
                // Check if the projection of the common point of the two
                // edges on the edge with the two other points is small
                Point_2 common_point = prev_edge.second;
                Point_2 other_point_1 = prev_edge.first;
                Point_2 other_point_2 = curr_edge.second;
                Segment_2 new_segment(other_point_1, other_point_2);
                double new_segment_length =
                    std::sqrt(new_segment.squared_length());
                Line_2 new_line(other_point_1, other_point_2);
                Point_2 projection = new_line.projection(common_point);
                double distance_to_projection =
                    std::sqrt(CGAL::squared_distance(projection, common_point));
                if (distance_to_projection < 0.1 * new_segment_length) {
                    // Simplify
                    edges.back().second = curr_edge.second;
                } else {
                    // Don't simplify
                    edges.push_back(curr_edge);
                }
            } else {
                edges.push_back(curr_edge);
            }
        }

        // Merge the last edge with the first edge if they are collinear
        auto &first_edge = edges.front();
        const auto &last_edge = edges.back();
        CustomCGAL::Angle angle = get_angle_between(first_edge, last_edge);
        if (angle.in_degrees() < ANGLE_THRESHOLD_DEGREES) {
            // Check if the projection of the common point of the two edges
            // on the edge with the two other points is small
            Point_2 common_point = last_edge.second;
            Point_2 other_point_1 = last_edge.first;
            Point_2 other_point_2 = first_edge.second;
            Segment_2 new_segment(other_point_1, other_point_2);
            double new_segment_length = std::sqrt(new_segment.squared_length());
            Line_2 new_line(other_point_1, other_point_2);
            Point_2 projection = new_line.projection(common_point);
            double distance_to_projection =
                std::sqrt(CGAL::squared_distance(projection, common_point));
            if (distance_to_projection < 0.1 * new_segment_length) {
                // Simplify
                first_edge.first = last_edge.first;
                edges.pop_back();
            }
        }
    }

    // Create a mapping of edges in order of decreasing length to process
    // the longest edges first
    std::vector<double> edge_lengths(edges.size());
    std::vector<size_t> edge_indices_ordered(edges.size());
    for (size_t i = 0; i < edges.size(); ++i) {
        edge_indices_ordered[i] = i;
        edge_lengths[i] =
            std::sqrt(CGAL::squared_distance(edges[i].first, edges[i].second));
    }
    std::sort(edge_indices_ordered.begin(), edge_indices_ordered.end(),
              [&edge_lengths](size_t i1, size_t i2) {
                  return edge_lengths[i1] > edge_lengths[i2];
              });

    // For each edge of the outline, compute the best offset using the
    // points in the buffer
    std::vector<Line_2> final_lines(edges.size());
    // Vector_2 current_average_shift(0, 0);
    // double total_length_processed = 0.0;
    for (size_t idx : edge_indices_ordered) {
        const auto &edge = edges[idx];
        Point_2 start_2d = edge.first;
        Point_2 end_2d = edge.second;
        // start_2d += current_average_shift;
        // end_2d += current_average_shift;
        Vector_2 edge_dir = end_2d - start_2d;
        Vector_2 offset_dir = edge_dir.perpendicular(CGAL::CLOCKWISE);
        offset_dir = offset_dir / std::sqrt(offset_dir.squared_length());

        auto [offset, fitted_segment] =
            best_proximity_offset(points_2d, weights, start_2d, end_2d,
                                  offset_dir, MAX_OFFSET, INTERVAL);

        // // Update the current average shift as an average of the shifts
        // weighted
        // // by edge length
        // double new_total_length_processed =
        //     total_length_processed + edge_lengths[idx];
        // current_average_shift =
        //     (current_average_shift * total_length_processed +
        //      offset * offset_dir * edge_lengths[idx]) /
        //     new_total_length_processed;
        // total_length_processed = new_total_length_processed;

        // Store the final line for this edge
        // final_lines[idx] = Line_2(new_start_2d, new_end_2d);
        final_lines[idx] = fitted_segment.supporting_line();
    }

    // Construct the roofprint by intersecting the successive lines
    std::vector<Point_2> roofprint_points;
    for (size_t i = 0; i < final_lines.size(); ++i) {
        const auto &line1 = final_lines[i];
        const auto &line2 = final_lines[(i + 1) % final_lines.size()];
        auto result = CGAL::intersection(line1, line2);
        if (const Point_2 *intersection_point =
                std::get_if<Point_2>(&*result)) {
            roofprint_points.push_back(*intersection_point);
        } else {
            // There was no intersection point
            std::cerr << "Warning: Lines " << line1 << " and " << line2
                      << " do not intersect at a single point." << std::endl;
            // As a fallback, we can take the end point of line1 and the
            // start point of line2, which should be close to the
            // intersection
            roofprint_points.push_back(line1.point(1)); // end point of line1
            roofprint_points.push_back(line2.point(0)); // start point of line2
        }
    }

    OGRPolygon *roofprint_polygon = new OGRPolygon();
    if (!roofprint_points.empty()) {
        OGRLinearRing *ring = new OGRLinearRing();
        for (const auto &p : roofprint_points) {
            ring->addPoint(p.x(), p.y());
        }
        // Close the ring by adding the first point at the end
        ring->addPoint(roofprint_points[0].x(), roofprint_points[0].y());
        roofprint_polygon->addRing(ring);
    }

    return PolygonZWithAttributes(OGRPolygonPtr(roofprint_polygon),
                                  outline_with_points.outline.id,
                                  outline_with_points.outline.outline_source);
}

MultiLineStringZWithAttributes
compute_roofprint_segments(const OutlineWithPoints &outline_with_points,
                           PtsStructs::StoragePtr points) {
    const auto &points_ids = outline_with_points.points_in_buffer;

    // Project the points in 2D in CGAL format
    std::vector<Point_2> points_2d(points_ids.size());
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::min();
    for (size_t i = 0; i < points_ids.size(); ++i) {
        const PtsStructs::PointId point_id = points_ids[i];
        const auto &point = points->get_point(point_id);
        points_2d[i] = Point_2(point.x(), point.y());
        min_z = std::min(min_z, point.z());
        max_z = std::max(max_z, point.z());
    }

    // Compute the weights for each point
    std::vector<double> weights(points_ids.size());
    for (size_t i = 0; i < points_ids.size(); ++i) {
        const PtsStructs::PointId point_id = points_ids[i];

        // Give more weight to higher points
        double z =
            points->get_field_as<double>(pdal::Dimension::Id::Z, point_id);
        double height_norm = (z - min_z + 1e-6) / (max_z - min_z + 1e-6);
        double height_factor = 1.0 + 2.0 * height_norm;

        // Give more weight to non-generated points
        uint8_t is_generated = points->get_field_as<uint8_t>(
            CustomDimensions::Id::IsGenerated, point_id);
        double generated_factor = 3 - is_generated;

        // Give more weight to points classified as building
        const auto cls_raw = points->get_field_as<
            std::underlying_type_t<LASclassification::Value>>(
            pdal::Dimension::Id::Classification, point_id);
        const auto cls = static_cast<LASclassification::Value>(cls_raw);
        double class_factor = 1.0;
        if (cls == LASclassification::Value::Building) {
            class_factor = 2.0;
        }

        // Combine the factors to get the final weight
        weights[i] = height_factor * generated_factor * class_factor;
    }

    // Find edges of the input outlines
    std::vector<std::pair<Point_2, Point_2>> initial_edges;
    const auto &polygon = outline_with_points.outline;
    const auto &exterior_ring = polygon.polygon->getExteriorRing();
    const auto num_points = exterior_ring->getNumPoints();
    for (int i = 0; i < num_points; ++i) {
        OGRPoint start, end;
        exterior_ring->getPoint(i, &start);
        exterior_ring->getPoint((i + 1) % num_points, &end);
        initial_edges.emplace_back(Point_2(start.getX(), start.getY()),
                                   Point_2(end.getX(), end.getY()));
    }

    // Simplify edges
    std::vector<std::pair<Point_2, Point_2>> edges;
    if (!initial_edges.empty()) {
        edges.push_back(initial_edges[0]);
        // Merge collinear edges
        for (size_t i = 1; i < initial_edges.size(); ++i) {
            const auto &prev_edge = edges.back();
            const auto &curr_edge = initial_edges[i];

            // // Get the angle between the two edges
            // CustomCGAL::Angle angle = get_angle_between(prev_edge,
            // curr_edge); if (std::abs(angle.in_degrees()) < 5.0) {
            //     // The edges are collinear, merge them by updating the
            //     end point edges.back().second = curr_edge.second;
            // } else {
            //     edges.push_back(curr_edge);
            // }

            CustomCGAL::Angle angle = get_angle_between(prev_edge, curr_edge);
            if (angle.in_degrees() < ANGLE_THRESHOLD_DEGREES) {
                // Check if the projection of the common point of the two
                // edges on the edge with the two other points is small
                Point_2 common_point = prev_edge.second;
                Point_2 other_point_1 = prev_edge.first;
                Point_2 other_point_2 = curr_edge.second;
                Segment_2 new_segment(other_point_1, other_point_2);
                double new_segment_length =
                    std::sqrt(new_segment.squared_length());
                Line_2 new_line(other_point_1, other_point_2);
                Point_2 projection = new_line.projection(common_point);
                double distance_to_projection =
                    std::sqrt(CGAL::squared_distance(projection, common_point));
                if (distance_to_projection < 0.1 * new_segment_length) {
                    // Simplify
                    edges.back().second = curr_edge.second;
                } else {
                    // Don't simplify
                    edges.push_back(curr_edge);
                }
            } else {
                edges.push_back(curr_edge);
            }
        }

        // Merge the last edge with the first edge if they are collinear
        auto &first_edge = edges.front();
        const auto &last_edge = edges.back();
        CustomCGAL::Angle angle = get_angle_between(first_edge, last_edge);
        if (angle.in_degrees() < ANGLE_THRESHOLD_DEGREES) {
            // Check if the projection of the common point of the two edges
            // on the edge with the two other points is small
            Point_2 common_point = last_edge.second;
            Point_2 other_point_1 = last_edge.first;
            Point_2 other_point_2 = first_edge.second;
            Segment_2 new_segment(other_point_1, other_point_2);
            double new_segment_length = std::sqrt(new_segment.squared_length());
            Line_2 new_line(other_point_1, other_point_2);
            Point_2 projection = new_line.projection(common_point);
            double distance_to_projection =
                std::sqrt(CGAL::squared_distance(projection, common_point));
            if (distance_to_projection < 0.1 * new_segment_length) {
                // Simplify
                first_edge.first = last_edge.first;
                edges.pop_back();
            }
        }
    }

    // Create a mapping of edges in order of decreasing length to process
    // the longest edges first
    std::vector<double> edge_lengths(edges.size());
    std::vector<size_t> edge_indices_ordered(edges.size());
    for (size_t i = 0; i < edges.size(); ++i) {
        edge_indices_ordered[i] = i;
        edge_lengths[i] =
            std::sqrt(CGAL::squared_distance(edges[i].first, edges[i].second));
    }
    std::sort(edge_indices_ordered.begin(), edge_indices_ordered.end(),
              [&edge_lengths](size_t i1, size_t i2) {
                  return edge_lengths[i1] > edge_lengths[i2];
              });

    // For each edge of the outline, compute the best offset using the
    // points in the buffer
    std::vector<Segment_2> final_segments(edges.size());
    // Vector_2 current_average_shift(0, 0);
    // double total_length_processed = 0.0;
    for (size_t idx : edge_indices_ordered) {
        const auto &edge = edges[idx];
        Point_2 start_2d = edge.first;
        Point_2 end_2d = edge.second;
        // start_2d += current_average_shift;
        // end_2d += current_average_shift;
        Vector_2 edge_dir = end_2d - start_2d;
        Vector_2 offset_dir = edge_dir.perpendicular(CGAL::CLOCKWISE);
        offset_dir = offset_dir / std::sqrt(offset_dir.squared_length());

        auto [offset, fitted_segment] =
            best_proximity_offset(points_2d, weights, start_2d, end_2d,
                                  offset_dir, MAX_OFFSET, INTERVAL);

        // // Update the current average shift as an average of the shifts
        // weighted
        // // by edge length
        // double new_total_length_processed =
        //     total_length_processed + edge_lengths[idx];
        // current_average_shift =
        //     (current_average_shift * total_length_processed +
        //      offset * offset_dir * edge_lengths[idx]) /
        //     new_total_length_processed;
        // total_length_processed = new_total_length_processed;

        // Store the final line for this edge
        // final_lines[idx] = Line_2(new_start_2d, new_end_2d);
        final_segments[idx] = fitted_segment;
    }

    MultiLineStringZ roofprint_multilinestring;
    for (const auto &segment : final_segments) {
        Segment_3 segment_3d(
            Point_3(segment.source().x(), segment.source().y(), 0),
            Point_3(segment.target().x(), segment.target().y(), 0));
        roofprint_multilinestring.add_line(segment_3d);
    }
    return MultiLineStringZWithAttributes(
        roofprint_multilinestring, outline_with_points.outline.id,
        outline_with_points.outline.outline_source);
}

MultiLineStringZWithAttributes
compute_roofprint_3d(const OutlineWithPoints &outline_with_points,
                     PtsStructs::StoragePtr points) {
    const auto &points_with_attr = outline_with_points.points_in_buffer;

    // Get the points in CGAL format
    std::vector<Point_3> points_3d(points_with_attr.size());
    for (size_t i = 0; i < points_with_attr.size(); ++i) {
        const auto &point = points->get_point(points_with_attr[i]);
        points_3d[i] = Point_3(point.x(), point.y(), point.z());
    }

    // Find edges with RANSAC
    RANSAC3DFitter ransac(points_3d, 1000, 5, 0.5, 10);
    std::vector<RansacLine> lines = ransac.run();

    MultiLineStringZ roofprint_multilinestring;
    for (const auto &ransac_line : lines) {
        Segment_3 segment = ransac.extract_segment(ransac_line);
        roofprint_multilinestring.add_line(segment);
    }
    return MultiLineStringZWithAttributes(
        roofprint_multilinestring, outline_with_points.outline.id,
        outline_with_points.outline.outline_source);
}

void compute_roofprints_in_las(
    NewLasReader &las_reader,
    const std::vector<MultiPolygonZWithAttributes> &all_outlines,
    std::vector<MultiLineStringZWithAttributes> &roofprints,
    double las_buffer_distance, double outline_buffer_distance) {
    roofprints.clear();

    // Select the outlines that are relevant for the LAS file
    std::vector<MultiPolygonZWithAttributes> selected_outlines;
    select_outlines_in_las(las_reader, all_outlines, las_buffer_distance,
                           selected_outlines);
    std::cout << "Selected " << selected_outlines.size()
              << " outlines that are relevant for the LAS file." << std::endl;

    if (selected_outlines.empty()) {
        std::cout
            << "No relevant outlines found, skipping roofprint computation."
            << std::endl;
        return;
    }

    // For each selected outline, find the points that are relevant for it
    std::cout << "Selecting points for each selected outline..." << std::endl;
    std::vector<OutlineWithPoints> outlines_with_points;
    select_points_per_outlines(las_reader, selected_outlines,
                               outline_buffer_distance, outlines_with_points);

    long total_points_with_outlines = 0;
    for (const auto &outline_with_points : outlines_with_points) {
        total_points_with_outlines +=
            outline_with_points.points_in_buffer.size();
    }
    if (outlines_with_points.empty()) {
        std::cerr << "The size of outlines_with_points should be the same as "
                     "the size of selected_outlines, which is "
                  << selected_outlines.size() << ", but it is "
                  << outlines_with_points.size() << "." << std::endl;
        throw std::runtime_error(
            "No outlines with points found, cannot compute roofprints.");
    }
    std::cout << "Average points per outline: "
              << static_cast<double>(total_points_with_outlines) /
                     outlines_with_points.size()
              << std::endl;

    // Compute the roofprints for each outline with its associated points
    std::cout << "Computing roofprints for each outline with its associated "
                 "points..."
              << std::endl;
    ProgressBarTotal bar(
        outlines_with_points.size(), "Computing roofprints",
        indicators::option::ForegroundColor{indicators::Color::green});
    bar.start();
    roofprints.reserve(outlines_with_points.size());
    for (const auto &outline_with_points : outlines_with_points) {
        const auto roofprint =
            compute_roofprint_segments(outline_with_points, las_reader.points);
        roofprints.push_back(roofprint);
        bar.increment(1);
    }
    bar.finish();
}

void compute_roofprints_in_las_3d(
    NewLasReader &las_reader,
    const std::vector<MultiPolygonZWithAttributes> &all_outlines,
    std::vector<MultiLineStringZWithAttributes> &roofprints,
    double las_buffer_distance, double outline_buffer_distance) {
    roofprints.clear();

    // Select the outlines that are relevant for the LAS file
    std::vector<MultiPolygonZWithAttributes> selected_outlines;
    select_outlines_in_las(las_reader, all_outlines, las_buffer_distance,
                           selected_outlines);
    std::cout << "Selected " << selected_outlines.size()
              << " outlines that are relevant for the LAS file." << std::endl;

    if (selected_outlines.empty()) {
        std::cout
            << "No relevant outlines found, skipping roofprint computation."
            << std::endl;
        return;
    }

    // For each selected outline, find the points that are relevant for it
    std::cout << "Selecting points for each selected outline..." << std::endl;
    std::vector<OutlineWithPoints> outlines_with_points;
    select_points_per_outlines(las_reader, selected_outlines,
                               outline_buffer_distance, outlines_with_points);

    long total_points_with_outlines = 0;
    for (const auto &outline_with_points : outlines_with_points) {
        total_points_with_outlines +=
            outline_with_points.points_in_buffer.size();
    }
    if (outlines_with_points.empty()) {
        std::cerr << "The size of outlines_with_points should be the same as "
                     "the size of selected_outlines, which is "
                  << selected_outlines.size() << ", but it is "
                  << outlines_with_points.size() << "." << std::endl;
        throw std::runtime_error(
            "No outlines with points found, cannot compute roofprints.");
    }
    std::cout << "Average points per outline: "
              << static_cast<double>(total_points_with_outlines) /
                     outlines_with_points.size()
              << std::endl;

    // Compute the roofprints for each outline with its associated points
    std::cout << "Computing roofprints for each outline with its associated "
                 "points..."
              << std::endl;
    ProgressBarTotal bar(
        outlines_with_points.size(), "Computing roofprints",
        indicators::option::ForegroundColor{indicators::Color::green});
    bar.start();
    roofprints.reserve(outlines_with_points.size());
    for (const auto &outline_with_points : outlines_with_points) {
        const auto roofprint =
            compute_roofprint_3d(outline_with_points, las_reader.points);
        roofprints.push_back(roofprint);
        bar.increment(1);
    }
    bar.finish();
}

void compute_roofprints(const std::string &input_las_file,
                        const std::string &input_bd_topo_file,
                        const std::string &output_roofprints_file,
                        double las_buffer_distance,
                        double outline_buffer_distance, bool overwrite) {

    if (std::filesystem::exists(output_roofprints_file) && !overwrite) {
        throw std::runtime_error("Output file already exists: " +
                                 output_roofprints_file);
    }

    std::filesystem::create_directories(
        std::filesystem::path(output_roofprints_file).parent_path());

    // Read the LAS file and get the point view
    std::cout << "Reading LAS file..." << std::endl;
    NewLasReader las_reader(input_las_file);

    // Read the building outlines from the BD TOPO file
    std::vector<MultiPolygonZWithAttributes> all_outlines;
    auto status =
        read_building_outlines_from_bd_topo(input_bd_topo_file, all_outlines);
    if (!status.ok()) {
        std::cerr << "Error reading BD TOPO: " << status.ToString()
                  << std::endl;
        throw std::runtime_error(
            "Failed to read building outlines from BD TOPO");
    }
    std::cout << "Successfully read " << all_outlines.size()
              << " building outlines from BD TOPO." << std::endl;

    // Compute the roofprints
    std::vector<MultiLineStringZWithAttributes> roofprints;
    compute_roofprints_in_las(las_reader, all_outlines, roofprints,
                              las_buffer_distance, outline_buffer_distance);
    // std::vector<MultiLineStringZWithAttributes> roofprints;
    // compute_roofprints_in_las_3d(las_reader, all_outlines, roofprints,
    //                              las_buffer_distance,
    //                              outline_buffer_distance);
    std::cout << "Computed " << roofprints.size() << " roofprints."
              << std::endl;

    std::cout << "Transform Polygons into MultiPolygons" << std::endl;
    // std::vector<MultiPolygonZWithAttributes> roofprints_as_multipolygons;
    // for (const auto &roofprint : roofprints) {
    //     roofprints_as_multipolygons.emplace_back(roofprint);
    // }

    // Write the roofprints to a Parquet file
    std::cout << "Writing roofprints to Parquet file..." << std::endl;
    auto write_status =
        write_geoms_to_parquet(roofprints, output_roofprints_file, overwrite);
    // auto write_status =
    //     write_geoms_to_parquet(roofprints, output_roofprints_file,
    //     overwrite);
    if (!write_status.ok()) {
        std::cerr << "Error writing roofprints to Parquet: "
                  << write_status.ToString() << std::endl;
        throw std::runtime_error("Failed to write roofprints to Parquet");
    }
}