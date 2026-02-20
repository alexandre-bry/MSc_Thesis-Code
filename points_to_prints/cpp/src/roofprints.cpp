#include "roofprints.hpp"

#include <CGAL/Vector_2.h>
#include <cstddef>
#include <filesystem>
#include <ogr_geometry.h>
#include <vector>

#include "geometry.hpp"
#include "kd_tree.hpp"
#include "las.hpp"
#include "parquet.hpp"
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

void select_points_for_outline(
    CustomLasReader &las_reader,
    const std::vector<MultiPolygonZWithAttributes> &outlines,
    double buffer_distance,
    std::vector<OutlineWithPoints> &outlines_with_points) {
    outlines_with_points.clear();

    OGREnvelopePtr las_bounding_box = las_reader.bounding_box();

    // Build a KD-tree for efficient spatial queries on the LAS points
    Points3DWithAttributes points(las_reader.point_view());
    std::vector<Point_2> cgal_points;
    cgal_points.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        const auto &point = points[i];
        cgal_points.emplace_back(point.x, point.y);
    }
    KdTree_2 kd_tree(cgal_points);

    // For each outline, find the points that are within its bounding box plus a
    // buffer
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
    }
}

double compute_metric(const std::vector<double> &offsets, double t) {
    double score = 0.0;
    for (double offset : offsets) {
        double dist = std::abs(offset - t);
        score += (dist < 1.0) ? (1.0 - dist) : 0.0;
    }
    return score;
}

double best_proximity_offset(const std::vector<Point_2> &points,
                             const Point_2 &base,
                             const Vector_2 &perp_dir, // unit vector
                             double max_offset, double epsilon = 1e-6) {
    std::vector<double> offsets;

    for (const auto &p : points) {
        offsets.push_back(perp_dir * (p - base));
    }

    // Ternary search on t in [-max_offset, max_offset]
    double left = -max_offset, right = max_offset;
    while (right - left > epsilon) {
        double m1 = left + (right - left) / 3;
        double m2 = right - (right - left) / 3;

        double f1 = compute_metric(offsets, m1);
        double f2 = compute_metric(offsets, m2);

        if (f1 > f2) {
            right = m2;
        } else {
            left = m1;
        }
    }

    return (left + right) / 2;
}

double MAX_OFFSET = 2.0;

PolygonZWithAttributes
compute_roofprint(const OutlineWithPoints &outline_with_points) {
    const auto &points_with_attr = outline_with_points.points_in_buffer;

    // Project the points in 2D in CGAL format
    std::vector<Point_2> points_2d(points_with_attr.size());
    for (size_t i = 0; i < points_with_attr.size(); ++i) {
        points_2d[i] = Point_2(points_with_attr[i].x, points_with_attr[i].y);
    }

    // For each edge of the outline, compute the best offset using the points in
    // the buffer
    const auto &polygon = outline_with_points.outline;
    const auto &exterior_ring = polygon.polygon->getExteriorRing();
    std::vector<Line_2> final_lines;
    for (int i = 0; i < exterior_ring->getNumPoints() - 1; ++i) {
        OGRPoint start, end;
        exterior_ring->getPoint(i, &start);
        exterior_ring->getPoint(i + 1, &end);
        Point_2 start_2d(start.getX(), start.getY());
        Point_2 end_2d(end.getX(), end.getY());
        Vector_2 edge_dir = end_2d - start_2d;
        Vector_2 offset_dir = edge_dir.perpendicular(CGAL::CLOCKWISE);

        double offset = best_proximity_offset(
            points_2d, start_2d,
            offset_dir / std::sqrt(offset_dir.squared_length()), MAX_OFFSET);

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
            // As a fallback, we can take the end point of line1 and the start
            // point of line2, which should be close to the intersection
            roofprint_points.push_back(line1.point(1)); // end point of line1
            roofprint_points.push_back(line2.point(0)); // start point of line2
        }
    }

    OGRPolygon *roofprint_polygon = new OGRPolygon();
    if (!roofprint_points.empty()) {
        auto ring = roofprint_polygon->getExteriorRing();
        for (const auto &p : roofprint_points) {
            ring->addPoint(p.x(), p.y());
        }
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
    std::vector<OutlineWithPoints> outlines_with_points;
    select_points_for_outline(las_reader, selected_outlines,
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
    roofprints.reserve(outlines_with_points.size());
    for (const auto &outline_with_points : outlines_with_points) {
        const auto roofprint = compute_roofprint(outline_with_points);
        roofprints.push_back(roofprint);
    }
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