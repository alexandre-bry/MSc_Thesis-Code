#include "ransac.hpp"

#include <cmath>
#include <limits>

RANSAC3DFitter::RANSAC3DFitter(const std::vector<Point_3> &points,
                               int max_iterations,
                               double radius_for_second_point,
                               double distance_threshold,
                               int max_icp_iterations)
    : points(points), max_iterations(max_iterations),
      radius_for_second_point(radius_for_second_point),
      distance_threshold(distance_threshold),
      max_icp_iterations(max_icp_iterations), assigned(points.size(), false) {
    gen.seed(rd());
}

int RANSAC3DFitter::get_random_unassigned_point() {
    std::vector<int> unassigned_indices;
    for (std::size_t i = 0; i < assigned.size(); ++i) {
        if (!assigned[i]) {
            unassigned_indices.push_back(i);
        }
    }

    if (unassigned_indices.empty()) {
        return -1;
    }

    std::uniform_int_distribution<> dis(0, unassigned_indices.size() - 1);
    return unassigned_indices[dis(gen)];
}

std::vector<std::size_t>
RANSAC3DFitter::get_points_in_radius(std::size_t point_idx, double radius) {
    std::vector<std::size_t> indices_in_radius;
    const Point_3 &center_point = points[point_idx];
    double radius_squared = radius * radius;

    for (std::size_t i = 0; i < points.size(); ++i) {
        if (i == point_idx || assigned[i]) {
            continue;
        }

        double dist_squared = CGAL::squared_distance(center_point, points[i]);
        if (dist_squared <= radius_squared) {
            indices_in_radius.push_back(i);
        }
    }

    return indices_in_radius;
}

std::vector<std::size_t>
RANSAC3DFitter::get_points_close_to_line(const Line_3 &line) {
    std::vector<std::size_t> close_points;
    double distance_threshold_squared = distance_threshold * distance_threshold;

    for (std::size_t i = 0; i < points.size(); ++i) {
        if (assigned[i]) {
            continue;
        }

        double dist_squared = CGAL::squared_distance(line, points[i]);
        if (dist_squared <= distance_threshold_squared) {
            close_points.push_back(i);
        }
    }

    return close_points;
}

Line_3 RANSAC3DFitter::optimize_line_with_icp(
    const Line_3 &initial_line, const std::vector<std::size_t> &point_indices) {
    if (point_indices.size() < 2) {
        return initial_line;
    }

    Line_3 current_line = initial_line;
    const double convergence_threshold = 1e-6;

    for (int iter = 0; iter < max_icp_iterations; ++iter) {
        // Step 1: Compute centroid of all points
        Vector_3 centroid(0, 0, 0);
        for (std::size_t idx : point_indices) {
            const Point_3 &p = points[idx];
            centroid = centroid + Vector_3(p.x(), p.y(), p.z());
        }
        centroid = centroid / static_cast<double>(point_indices.size());
        Point_3 centroid_point(centroid.x(), centroid.y(), centroid.z());

        // Step 2: Compute covariance matrix for PCA
        // We need to find the direction of maximum variance
        double xx = 0, xy = 0, xz = 0;
        double yy = 0, yz = 0, zz = 0;

        for (std::size_t idx : point_indices) {
            const Point_3 &p = points[idx];
            double dx = p.x() - centroid_point.x();
            double dy = p.y() - centroid_point.y();
            double dz = p.z() - centroid_point.z();

            xx += dx * dx;
            xy += dx * dy;
            xz += dx * dz;
            yy += dy * dy;
            yz += dy * dz;
            zz += dz * dz;
        }

        // Simple power iteration to find principal eigenvector
        Vector_3 direction(1, 0, 0);
        for (int pi = 0; pi < 20; ++pi) {
            double nx =
                xx * direction.x() + xy * direction.y() + xz * direction.z();
            double ny =
                xy * direction.x() + yy * direction.y() + yz * direction.z();
            double nz =
                xz * direction.x() + yz * direction.y() + zz * direction.z();

            double norm = std::sqrt(nx * nx + ny * ny + nz * nz);
            if (norm > 1e-10) {
                direction = Vector_3(nx / norm, ny / norm, nz / norm);
            }
        }

        // Step 3: Create new line through centroid with best-fit direction
        Line_3 new_line(centroid_point, direction);

        // Step 4: Check convergence
        // Compare the directions and positions
        Vector_3 old_dir = current_line.to_vector();
        Vector_3 new_dir = new_line.to_vector();
        // Normalize directions for comparison
        old_dir = old_dir / std::sqrt(old_dir.squared_length());
        new_dir = new_dir / std::sqrt(new_dir.squared_length());

        // Compute angular difference (using dot product)
        double dot_product = std::abs(old_dir * new_dir);
        if (dot_product > 1.0 - convergence_threshold) {
            // Converged
            return new_line;
        }

        current_line = new_line;
    }

    return current_line;
}

std::vector<RansacLine> RANSAC3DFitter::run() {
    std::vector<RansacLine> fitted_lines;

    int iteration = 0;
    while (iteration < max_iterations) {
        // Get a random unassigned point as the first point
        int first_point_idx = get_random_unassigned_point();
        if (first_point_idx == -1) {
            // No more unassigned points
            break;
        }

        // Find points within the radius from the first point
        std::vector<std::size_t> candidates_for_second =
            get_points_in_radius(first_point_idx, radius_for_second_point);

        if (candidates_for_second.empty()) {
            // No points found within radius, mark first point as assigned
            // and move to next iteration
            assigned[first_point_idx] = true;
            iteration++;
            continue;
        }

        // Pick a random point from the candidates as the second point
        std::uniform_int_distribution<> dis(0,
                                            candidates_for_second.size() - 1);
        std::size_t second_point_idx = candidates_for_second[dis(gen)];

        // Create a line from the two points
        Line_3 fitted_line(points[first_point_idx], points[second_point_idx]);

        // Find all unassigned points close to this line
        std::vector<std::size_t> points_on_line =
            get_points_close_to_line(fitted_line);

        // Always include the two points that defined the line
        points_on_line.push_back(first_point_idx);
        points_on_line.push_back(second_point_idx);

        // Remove duplicates
        std::sort(points_on_line.begin(), points_on_line.end());
        points_on_line.erase(
            std::unique(points_on_line.begin(), points_on_line.end()),
            points_on_line.end());

        // Optimize the line using ICP to get a better fit
        Line_3 optimized_line =
            optimize_line_with_icp(fitted_line, points_on_line);

        // After optimization, re-check which points are close to the optimized
        // line First, unassign the points that were on the initial line
        for (std::size_t idx : points_on_line) {
            assigned[idx] = false;
        }

        // Find points close to the optimized line
        std::vector<std::size_t> updated_points_on_line =
            get_points_close_to_line(optimized_line);

        // Always include the two points that originally defined the line
        updated_points_on_line.push_back(first_point_idx);
        updated_points_on_line.push_back(second_point_idx);

        // Remove duplicates
        std::sort(updated_points_on_line.begin(), updated_points_on_line.end());
        updated_points_on_line.erase(std::unique(updated_points_on_line.begin(),
                                                 updated_points_on_line.end()),
                                     updated_points_on_line.end());

        // Mark all points on the optimized line as assigned
        for (std::size_t idx : updated_points_on_line) {
            assigned[idx] = true;
        }

        // Calculate a score (number of points on the line)
        double score = static_cast<double>(updated_points_on_line.size());

        // Add the optimized line to the results
        fitted_lines.push_back(
            RansacLine{optimized_line, updated_points_on_line, score});

        iteration++;
    }

    return fitted_lines;
}

Segment_3 RANSAC3DFitter::extract_segment(const RansacLine &ransac_line) const {
    if (ransac_line.point_indices.empty()) {
        // Return a degenerate segment if no points
        Point_3 p = ransac_line.line.point(0);
        return Segment_3(p, p);
    }

    // Project all points onto the line and find min/max extents
    const Line_3 &line = ransac_line.line;
    Point_3 line_point = line.point(0);
    Vector_3 line_direction = line.to_vector();

    // Normalize the direction vector
    double length = std::sqrt(line_direction.squared_length());
    if (length < 1e-10) {
        // Degenerate line, return a point
        Point_3 p = points[ransac_line.point_indices[0]];
        return Segment_3(p, p);
    }
    Vector_3 unit_direction = line_direction / length;

    double min_t = std::numeric_limits<double>::max();
    double max_t = std::numeric_limits<double>::lowest();

    // For each point, compute its projection parameter t along the line
    for (std::size_t idx : ransac_line.point_indices) {
        const Point_3 &p = points[idx];
        Vector_3 point_to_line_point = p - line_point;

        // Project onto the line: t = (p - line_point) · unit_direction
        double t = point_to_line_point * unit_direction;

        min_t = std::min(min_t, t);
        max_t = std::max(max_t, t);
    }

    // Construct the segment endpoints
    Point_3 start = line_point + min_t * unit_direction;
    Point_3 end = line_point + max_t * unit_direction;

    return Segment_3(start, end);
}
