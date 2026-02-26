#include "roofprints.hpp"

#include <CGAL/Kernel/global_functions_3.h>
#include <CGAL/Segment_2.h>
#include <CGAL/Vector_2.h>
#include <cstddef>
#include <filesystem>
#include <iostream>
#include <ogr_geometry.h>
#include <vector>

#include "cgal.hpp"
#include "geometry.hpp"
#include "kd_tree.hpp"
#include "las.hpp"
#include "parquet.hpp"
#include "pbar.hpp"
#include "points.hpp"

void select_outlines_in_las(
    CustomLasReader &reader,
    const std::vector<MultiPolygonZWithAttributes> &outlines,
    double buffer_distance,
    std::vector<MultiPolygonZWithAttributes> &selected_outlines) {
    selected_outlines.clear();

    OGREnvelopePtr las_bounding_box = reader.bounding_box();
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
    CustomLasReader &las_reader,
    const std::vector<MultiPolygonZWithAttributes> &outlines,
    double buffer_distance,
    std::vector<OutlineWithPoints> &outlines_with_points) {
    outlines_with_points.clear();

    OGREnvelopePtr las_bounding_box = las_reader.bounding_box();

    // Build a KD-tree for efficient spatial queries on the LAS points
    std::cout << "Building KD-tree for LAS points..." << std::endl;
    Points3DWithAttributes points(las_reader.point_view());
    std::cout << "Number of points: " << points.points.size() << std::endl;
    std::vector<Point_2> cgal_points;
    cgal_points.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        const auto &point = points[i];
        cgal_points.emplace_back(point.x, point.y);
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

            std::vector<Point3DWithAttributes> points_on_outline;
            std::vector<std::size_t> indices_in_area =
                kd_tree.search_indices_in_box({bbox->MinX, bbox->MinY},
                                              {bbox->MaxX, bbox->MaxY});
            for (std::size_t idx : indices_in_area) {
                points_on_outline.push_back(points[idx]);
            }
            outlines_with_points.emplace_back(polygon, points_on_outline);
        }
        bar.increment(1);
    }
    bar.finish();
}

const double METRIC_INTERVAL = 0.3;

double compute_metric(const std::vector<double> &offsets, double t) {
    double total_score = 0.0;
    for (double offset : offsets) {
        double dist = std::abs(offset - t);
        double score =
            (dist < METRIC_INTERVAL) ? (METRIC_INTERVAL - dist) : 0.0;
        total_score += std::sqrt(score);
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

double best_proximity_offset(const std::vector<Point_2> &points,
                             const Point_2 &base_start, const Point_2 &base_end,
                             const Vector_2 &perp_dir, double max_offset,
                             double offset_step) {
    double best_offset = 0.0;
    double best_score = 0.0;

    Vector_2 start_to_end = base_end - base_start;
    double start_to_end_length = CGAL::sqrt(start_to_end.squared_length());
    Vector_2 start_to_end_unit = start_to_end / start_to_end_length;

    std::vector<double> offsets;
    for (const auto &p : points) {
        // Check if the point projected on the edge is within the edge
        // segment
        Vector_2 start_to_p = p - base_start;
        double projection_length = start_to_p * start_to_end_unit;
        if (projection_length < 0 || projection_length > start_to_end_length) {
            continue;
        }

        double perp_distance = perp_dir * (p - base_start);
        offsets.push_back(perp_distance);
    }

    for (double offset = -max_offset; offset <= max_offset;
         offset += offset_step) {
        double score = compute_metric(offsets, offset);
        if (score > best_score) {
            best_score = score;
            best_offset = offset;
        }
    }

    return best_offset;
}

double get_angle_degrees_between(std::pair<Point_2, Point_2> edge1,
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
    return angle;
}

double MAX_OFFSET = 2.0;
double INTERVAL = 0.001;

PolygonZWithAttributes
compute_roofprint(const OutlineWithPoints &outline_with_points) {
    const auto &points_with_attr = outline_with_points.points_in_buffer;

    // Project the points in 2D in CGAL format
    std::vector<Point_2> points_2d(points_with_attr.size());
    for (size_t i = 0; i < points_with_attr.size(); ++i) {
        points_2d[i] = Point_2(points_with_attr[i].x, points_with_attr[i].y);
    }

    // Find edges of the input outlines and merge the ones that are collinear
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

    std::vector<std::pair<Point_2, Point_2>> edges;
    if (!initial_edges.empty()) {
        edges.push_back(initial_edges[0]);
        // Merge collinear edges
        for (size_t i = 1; i < initial_edges.size(); ++i) {
            const auto &prev_edge = edges.back();
            const auto &curr_edge = initial_edges[i];

            // Get the angle between the two edges
            double angle = get_angle_degrees_between(prev_edge, curr_edge);
            if (std::abs(angle) < 5.0) {
                // The edges are collinear, merge them by updating the end point
                edges.back().second = curr_edge.second;
            } else {
                edges.push_back(curr_edge);
            }
        }

        // Merge the last edge with the first edge if they are collinear
        auto &first_edge = edges.front();
        const auto &last_edge = edges.back();
        double angle = get_angle_degrees_between(first_edge, last_edge);
        if (std::abs(angle) < 5.0) {
            first_edge.first = last_edge.first;
            edges.pop_back();
        }
    }

    // For each edge of the outline, compute the best offset using the points in
    // the buffer
    std::vector<Line_2> final_lines;
    for (size_t i = 0; i < edges.size(); ++i) {
        const auto &edge = edges[i];
        Point_2 start_2d = edge.first;
        Point_2 end_2d = edge.second;
        Vector_2 edge_dir = end_2d - start_2d;
        Vector_2 offset_dir = edge_dir.perpendicular(CGAL::CLOCKWISE);
        offset_dir = offset_dir / std::sqrt(offset_dir.squared_length());

        double offset = best_proximity_offset(points_2d, start_2d, end_2d,
                                              offset_dir, MAX_OFFSET, INTERVAL);

        final_lines.emplace_back(start_2d + offset * offset_dir,
                                 end_2d + offset * offset_dir);
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

void compute_roofprints_in_las(
    CustomLasReader &las_reader,
    const std::vector<MultiPolygonZWithAttributes> &all_outlines,
    std::vector<PolygonZWithAttributes> &roofprints, double las_buffer_distance,
    double outline_buffer_distance) {
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
        const auto roofprint = compute_roofprint(outline_with_points);
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
    CustomLasReader las_reader(input_las_file);
    las_reader.execute();

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
    std::vector<PolygonZWithAttributes> roofprints;
    compute_roofprints_in_las(las_reader, all_outlines, roofprints,
                              las_buffer_distance, outline_buffer_distance);
    std::cout << "Computed " << roofprints.size() << " roofprints."
              << std::endl;

    std::cout << "Transform Polygons into MultiPolygons" << std::endl;
    std::vector<MultiPolygonZWithAttributes> roofprints_as_multipolygons;
    for (const auto &roofprint : roofprints) {
        roofprints_as_multipolygons.emplace_back(roofprint);
    }

    // Write the roofprints to a Parquet file
    std::cout << "Writing roofprints to Parquet file..." << std::endl;
    auto write_status = write_multi_polygons_to_parquet(
        roofprints_as_multipolygons, output_roofprints_file, overwrite);
    if (!write_status.ok()) {
        std::cerr << "Error writing roofprints to Parquet: "
                  << write_status.ToString() << std::endl;
        throw std::runtime_error("Failed to write roofprints to Parquet");
    }
}