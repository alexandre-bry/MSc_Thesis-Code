#include "roofprints_new.hpp"

#include "constants.hpp"
#include "points.hpp"
#include "utils/cgal.hpp"

Roofprints::AllPoints::AllPoints(const std::vector<Point_2> &points)
    : points(points) {}

Point_2 Roofprints::AllPoints::get(PointId point_id) const {
    if (point_id < 0 || point_id >= points.size()) {
        throw std::out_of_range("Point ID out of range: " +
                                std::to_string(point_id));
    }
    return points[point_id];
}

void Roofprints::AllPoints::set(PointId point_id, Point_2 point) {
    if (point_id < 0 || point_id >= points.size()) {
        throw std::out_of_range("Point ID out of range: " +
                                std::to_string(point_id));
    }
    points[point_id] = point;
}

Roofprints::EdgeSequence::EdgeSequence(const std::vector<PointId> &point_ids)
    : point_ids(point_ids) {}

void Roofprints::EdgeSequence::compute_updated_points(
    Point_2 new_start, Point_2 new_end, AllPointsPtr initial_points_ptr,
    AllPointsPtr new_points_ptr, std::vector<Point_2> &new_points) {
    PointId start_id = get_start();
    PointId end_id = get_end();
    Point_2 initial_start = initial_points_ptr->get(start_id);
    Point_2 initial_end = initial_points_ptr->get(end_id);
    Vector_2 initial_vec = initial_end - initial_start;
    Vector_2 new_vec = new_end - new_start;

    // Check if the old and the new vectors are almost parallel
    if (!CustomCGAL::are_almost_parallel(initial_vec, new_vec,
                                         CustomCGAL::Angle::from_degrees(1))) {
        throw std::runtime_error(
            "Cannot update inner points of edge sequence because the movement "
            "is not in the same direction as the original edge");
    }

    // Compute the shift and the scale factor of the movement
    double initial_length = std::sqrt(initial_vec.squared_length());
    double new_length = std::sqrt(new_vec.squared_length());
    if (initial_length == 0) {
        throw std::runtime_error(
            "Cannot update inner points of edge sequence because the initial "
            "length of the edge is zero");
    }
    double scale_factor = new_length / initial_length;
    Vector_2 shift = new_start - initial_start;

    // Update the start and end points
    new_points_ptr->set(start_id, new_start);
    new_points_ptr->set(end_id, new_end);

    // Update the inner points
    for (std::size_t i = 1; i < point_ids.size() - 1; ++i) {
        PointId point_id = point_ids[i];
        Point_2 initial_point = initial_points_ptr->get(point_id);
        Vector_2 initial_point_vec = initial_point - initial_start;
        Point_2 new_point =
            new_start + initial_point_vec * scale_factor + shift;
        new_points_ptr->set(point_id, new_point);
    }
}

Roofprints::PointId Roofprints::EdgeSequence::get_start() const {
    if (point_ids.empty()) {
        throw std::runtime_error("Edge group has no points");
    }
    return point_ids.front();
}

Roofprints::PointId Roofprints::EdgeSequence::get_end() const {
    if (point_ids.empty()) {
        throw std::runtime_error("Edge group has no points");
    }
    return point_ids.back();
}

Roofprints::EdgeSequenceWithNeighbours::EdgeSequenceWithNeighbours(
    const EdgeSequence &main, const EdgeSequence &previous,
    const EdgeSequence &next)
    : main(main), previous(previous), next(next) {
    if (main.get_start() != previous.get_end()) {
        throw std::runtime_error("Previous edge sequence is not connected to "
                                 "the main edge sequence");
    }
    if (main.get_end() != next.get_start()) {
        throw std::runtime_error(
            "Next edge sequence is not connected to the main edge sequence");
    }
}

std::tuple<double, double>
Roofprints::EdgeSequenceWithNeighbours::compute_movement_limits(
    Vector_2 moving_direction, AllPointsPtr all_points_ptr) const {
    // Compute the maximum and minimum offset for the moving direction.
    // The constraint for keeping the geometry valid is that we can move the
    // main edges in a direction as long as none of the edges is reduced to a
    // point.

    // Check if the moving_direction is normalized
    if (std::abs(std::sqrt(moving_direction.squared_length()) - 1.0) > 1e-6) {
        throw std::runtime_error("Moving direction is not normalized");
    }

    Point_2 prev_start = all_points_ptr->get(previous.get_start());
    Point_2 main_start = all_points_ptr->get(main.get_start());
    Point_2 main_end = all_points_ptr->get(main.get_end());
    Point_2 next_end = all_points_ptr->get(next.get_end());

    // Check if the consecutive edges are almost collinear
    if (!CustomCGAL::are_almost_collinear(prev_start, main_start, main_end,
                                          CustomCGAL::Angle::from_degrees(5))) {
        throw std::runtime_error(
            "Previous edge sequence is not almost collinear with the main edge "
            "sequence");
    }
    if (!CustomCGAL::are_almost_collinear(main_start, main_end, next_end,
                                          CustomCGAL::Angle::from_degrees(5))) {
        throw std::runtime_error(
            "Next edge sequence is not almost collinear with the main edge "
            "sequence");
    }

    double min_limit = -std::numeric_limits<double>::infinity();
    double max_limit = std::numeric_limits<double>::infinity();

    // First constraint: the previous edge should not be reduced to a point
    Vector_2 previous_edge_vec = prev_start - main_start;
    double previous_edge_on_moving_dir = previous_edge_vec * moving_direction;
    if (previous_edge_on_moving_dir > 0) {
        max_limit = std::min(max_limit, previous_edge_on_moving_dir);
    } else {
        min_limit = std::max(min_limit, previous_edge_on_moving_dir);
    }

    // Second constraint: the next edge should not be reduced to a point
    Vector_2 next_edge_vec = next_end - main_end;
    double next_edge_on_moving_dir = next_edge_vec * moving_direction;
    if (next_edge_on_moving_dir > 0) {
        max_limit = std::min(max_limit, next_edge_on_moving_dir);
    } else {
        min_limit = std::max(min_limit, next_edge_on_moving_dir);
    }

    // Third constraint: the main edge should not be reduced to a point
    // Find the intersection point of the lines of the previous and next
    // edges
    Line_2 previous_line(prev_start, main_start);
    Line_2 next_line(main_end, next_end);
    auto intersection = CGAL::intersection(previous_line, next_line);
    if (intersection) {
        if (const Line_2 *intersection_line =
                std::get_if<Line_2>(&*intersection)) {
            throw std::runtime_error(
                "Previous and next edge sequences are collinear");
        } else if (const Point_2 *intersection_point =
                       std::get_if<Point_2>(&*intersection)) {
            Vector_2 start_to_intersection_vec =
                *intersection_point - main_start;
            double start_to_intersection_on_moving_dir =
                start_to_intersection_vec * moving_direction;
            if (start_to_intersection_on_moving_dir > 0) {
                max_limit =
                    std::min(max_limit, start_to_intersection_on_moving_dir);
            } else {
                min_limit =
                    std::max(min_limit, start_to_intersection_on_moving_dir);
            }
        } else {
            throw std::runtime_error("Unexpected intersection type");
        }
    }

    return {min_limit, max_limit};
}

void Roofprints::EdgeSequenceWithNeighbours::compute_metric_for_offsets(
    const Vector_2 &moving_direction, const std::vector<double> &offsets,
    AllPointsPtr points_outlines,
    const std::vector<PtsStructs::PointId> &point_cloud_point_ids,
    const std::vector<double> &point_cloud_point_weights,
    PtsStructs::StoragePtr points_point_cloud,
    std::vector<double> &metrics) const {
    // TODO: Also look at the neighbouring edges

    // TODO: Compute the metric efficiently for all offsets by projecting only
    // once and then checking whether the projected points are within the edge
    // segment for each offset

    // Extract the relevant points and directions
    Point_2 prev_start = points_outlines->get(previous.get_start());
    Point_2 main_start = points_outlines->get(main.get_start());
    Point_2 main_end = points_outlines->get(main.get_end());
    Point_2 next_end = points_outlines->get(next.get_end());
    Vector_2 prev_direction = main_start - prev_start;
    prev_direction =
        prev_direction / std::sqrt(prev_direction.squared_length());
    Vector_2 next_direction = next_end - main_end;
    next_direction =
        next_direction / std::sqrt(next_direction.squared_length());

    double prev_scale = prev_direction * moving_direction;
    double next_scale = next_direction * moving_direction;

    // Extract the points from the point cloud
    std::vector<Point_2> point_cloud_points(point_cloud_point_ids.size());
    for (PtsStructs::PointId i(0); i < point_cloud_point_ids.size(); ++i) {
        Point_3 point_3d =
            points_point_cloud->get_point(point_cloud_point_ids[i]);
        point_cloud_points[i] = Point_2(point_3d.x(), point_3d.y());
    }

    metrics.resize(offsets.size());
    for (size_t i = 0; i < offsets.size(); ++i) {
        double offset = offsets[i];
        Point_2 new_main_start =
            main_start + offset * prev_scale * prev_direction;
        Point_2 new_main_end = main_end + offset * next_scale * next_direction;

        Segment_2 new_main_edge(new_main_start, new_main_end);

        Vector_2 edge_vec = new_main_end - new_main_start;
        double edge_vec_length = std::sqrt(edge_vec.squared_length());

        double total_score = 0.0;
        for (size_t j = 0; j < point_cloud_point_ids.size(); ++j) {
            const Point_2 &point = point_cloud_points[j];

            // Check if the point projected on the edge is within the edge
            // segment
            Vector_2 start_to_point = point - new_main_start;
            double projection_length = start_to_point * edge_vec;
            if (projection_length < 0 || projection_length > edge_vec_length) {
                continue;
            }

            // Compute the distance from the point to the edge and the score
            // based on the distance
            double dist =
                std::sqrt(CGAL::squared_distance(new_main_edge, point));
            double score =
                (dist < METRIC_INTERVAL) ? (METRIC_INTERVAL - dist) : 0.0;
            total_score += score * point_cloud_point_weights[j];
        }
        metrics[i] = total_score / edge_vec_length;
    }
}

Roofprints::EdgeMover::EdgeMover(
    const std::vector<EdgeSequenceWithNeighbours> &edges_with_neighbours,
    const Vector_2 &moving_direction, AllPointsPtr all_points_ptr)
    : edges_with_neighbours(edges_with_neighbours),
      moving_direction(moving_direction), all_points_ptr(all_points_ptr) {}

std::tuple<double, double>
Roofprints::EdgeMover::_compute_movement_limits() const {
    double min_offset = -std::numeric_limits<double>::infinity();
    double max_offset = std::numeric_limits<double>::infinity();
    for (const auto &edge : edges_with_neighbours) {
        auto [edge_min_offset, edge_max_offset] =
            edge.compute_movement_limits(moving_direction, all_points_ptr);
        min_offset = std::max(min_offset, edge_min_offset);
        max_offset = std::min(max_offset, edge_max_offset);
    }
    return {min_offset, max_offset};
}

void Roofprints::EdgeMover::optimize(
    double max_absolute_offset, double offset_step,
    const std::vector<PtsStructs::PointId> &point_cloud_point_ids,
    const std::vector<double> &point_cloud_point_weights,
    PtsStructs::StoragePtr points_point_cloud) const {
    auto [min_offset_limit, max_offset_limit] = _compute_movement_limits();
    double actual_min_offset = std::max(min_offset_limit, -max_absolute_offset);
    double actual_max_offset = std::min(max_offset_limit, max_absolute_offset);

    // Build the list of offsets to evaluate
    std::vector<double> offsets(0.0);
    for (double offset = offset_step; offset <= actual_max_offset;
         offset += offset_step) {
        offsets.push_back(offset);
    }
    for (double offset = -offset_step; offset >= actual_min_offset;
         offset -= offset_step) {
        offsets.push_back(offset);
    }

    // Compute the metrics for all offsets
    std::vector<double> metrics_sum;
    for (const auto &edge_with_neighbours : edges_with_neighbours) {
        std::vector<double> metrics;
        edge_with_neighbours.compute_metric_for_offsets(
            moving_direction, offsets, all_points_ptr, point_cloud_point_ids,
            point_cloud_point_weights, points_point_cloud, metrics);
        if (metrics_sum.empty()) {
            metrics_sum = metrics;
        } else {
            for (size_t i = 0; i < metrics.size(); ++i) {
                metrics_sum[i] += metrics[i];
            }
        }
    }

    // Find the best offset
    double best_offset = 0.0;
    double best_metric = -std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < offsets.size(); ++i) {
        if (metrics_sum[i] > best_metric) {
            best_metric = metrics_sum[i];
            best_offset = offsets[i];
        }
    }

    // TODO: decide what to do with the offset
}

Roofprints::SuperOutline::SuperOutline(
    const std::vector<EdgeSequence> &edges,
    const std::vector<std::string> &edge_index_to_outline_id,
    const std::vector<unsigned long> &edge_index_to_edge_group_id,
    const std::vector<std::vector<EdgeSequenceId>>
        &edge_index_to_neighbour_edge_indices) {
    // TODO
}

void Roofprints::SuperOutline::optimize_edges(AllPointsPtr all_points_ptr) {
    // TODO
}