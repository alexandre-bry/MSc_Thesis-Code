#include "topology.hpp"

#include <cstdint>
#include <filesystem>
#include <vector>

#include "../las/reader.hpp"
#include "../parquet.hpp"
#include "../points.hpp"
#include "../utils/cgal.hpp"

AllLines::Edge::Edge(Point_3 initial_start, Point_3 initial_end)
    : initial_start(initial_start), initial_end(initial_end) {
    Point_2 start_2d(initial_start.x(), initial_start.y());
    Point_2 end_2d(initial_end.x(), initial_end.y());
    line = Line_2(start_2d, end_2d);
    direction = line.direction().to_vector();
    direction /= std::sqrt(direction.squared_length());
}

Point_3 AllLines::Edge::get_initial_start() const { return initial_start; }
Point_3 AllLines::Edge::get_initial_end() const { return initial_end; }
Vector_2 AllLines::Edge::get_direction() const { return direction; }
Line_2 AllLines::Edge::to_line() const { return line; }

void AllLines::Edge::translate(Vector_2 offset) { line = translated(offset); }

AllLines::Edge AllLines::Edge::translated(Vector_2 offset) const {
    Point_2 new_start = line.point(0) + offset;
    Point_2 new_end = line.point(1) + offset;
    return Edge(Point_3(new_start.x(), new_start.y(), initial_start.z()),
                Point_3(new_end.x(), new_end.y(), initial_end.z()));
}

AllLines::OutlineAsEdges::OutlineAsEdges(
    const std::vector<std::vector<EdgeId>> &polygons)
    : polygons(polygons) {}

AllLines::OptimizationUnit::OptimizationUnit(
    const std::vector<EdgeGroupId> &edge_group_ids)
    : edge_group_ids(edge_group_ids) {}

AllLines::AllOutlines::AllOutlines(
    const EdgeVector<Edge> &edges, const EdgeVector<EdgeId> &prev_edge_ids,
    const EdgeVector<EdgeId> &next_edge_ids,
    const EdgeGroupVector<EdgeGroup> &edge_groups,
    const EdgeVector<EdgeGroupId> &edge_id_to_edge_group_id,
    const OutlineVector<OutlineAsEdges> &outlines,
    const EdgeVector<OutlineId> &edge_id_to_outline_id,
    const OptimizationUnitVector<OptimizationUnit> &optim_units,
    const EdgeGroupVector<OptimizationUnitId> &edge_group_id_to_optim_unit_id)
    : edges(edges), prev_edge_ids(prev_edge_ids), next_edge_ids(next_edge_ids),
      edge_groups(edge_groups),
      edge_id_to_edge_group_id(edge_id_to_edge_group_id), outlines(outlines),
      edge_id_to_outline_id(edge_id_to_outline_id), optim_units(optim_units),
      edge_group_id_to_optim_unit_id(edge_group_id_to_optim_unit_id) {}

const AllLines::Edge &AllLines::AllOutlines::get_edge(EdgeId edge_id) const {
    return edges.at(edge_id);
}

AllLines::Edge &AllLines::AllOutlines::get_edge(EdgeId edge_id) {
    return edges.at(edge_id);
}

AllLines::EdgeGroupId
AllLines::AllOutlines::get_edge_group_id(EdgeId edge_id) const {
    return edge_id_to_edge_group_id.at(edge_id);
}

AllLines::OutlineId
AllLines::AllOutlines::get_outline_id(EdgeId edge_id) const {
    return edge_id_to_outline_id.at(edge_id);
}

AllLines::OptimizationUnitId
AllLines::AllOutlines::get_optim_unit_id(EdgeGroupId edge_group_id) const {
    return edge_group_id_to_optim_unit_id.at(edge_group_id);
}

AllLines::EdgeId AllLines::AllOutlines::get_prev_edge_id(EdgeId edge_id) const {
    return prev_edge_ids.at(edge_id);
}

AllLines::EdgeId AllLines::AllOutlines::get_next_edge_id(EdgeId edge_id) const {
    return next_edge_ids.at(edge_id);
}

Point_2 AllLines::AllOutlines::get_edge_start(EdgeId edge_id) const {
    EdgeId prev_edge_id = get_prev_edge_id(edge_id);
    Edge current_edge = get_edge(edge_id);
    Edge prev_edge = get_edge(prev_edge_id);
    return CustomCGAL::intersection(current_edge.to_line(),
                                    prev_edge.to_line());
}

Point_2 AllLines::AllOutlines::get_edge_end(EdgeId edge_id) const {
    EdgeId next_edge_id = get_next_edge_id(edge_id);
    Edge current_edge = get_edge(edge_id);
    Edge next_edge = get_edge(next_edge_id);
    return CustomCGAL::intersection(current_edge.to_line(),
                                    next_edge.to_line());
}

AllLines::EdgeId AllLines::AllOutlines::edge_count() const {
    return edges.size_as_strong_index();
}

AllLines::EdgeGroupId AllLines::AllOutlines::edge_group_count() const {
    return edge_groups.size_as_strong_index();
}

AllLines::OutlineId AllLines::AllOutlines::outline_count() const {
    return outlines.size_as_strong_index();
}

AllLines::OptimizationUnitId AllLines::AllOutlines::optim_unit_count() const {
    return optim_units.size_as_strong_index();
}

AllLines::AllOutlines AllLines::make_all_outlines(
    const EdgeVector<Edge> &edges,
    const OutlineVector<OutlineAsEdges> &outlines,
    const std::vector<std::pair<EdgeId, EdgeId>> &intersections) {

    /* -------------------------------- Edges
     * ------------------------------- */

    // Find the previous and next edge for each edge
    EdgeVector<EdgeId> prev_edge_ids(edges.size());
    EdgeVector<EdgeId> next_edge_ids(edges.size());
    for (const auto &outline : outlines) {
        if (outline.polygons.empty()) {
            throw std::runtime_error("Outline has no polygons");
        }
        for (const auto &polygon : outline.polygons) {
            if (polygon.empty()) {
                throw std::runtime_error("Polygon has no edges");
            }
            for (std::size_t i = 0; i < polygon.size(); ++i) {
                EdgeId edge_id = polygon[i];
                prev_edge_ids[edge_id] =
                    polygon[(i + polygon.size() - 1) % polygon.size()];
                next_edge_ids[edge_id] = polygon[(i + 1) % polygon.size()];
            }
        }
    }

    /* ------------------------ Edge groups
     * ------------------------ */

    // Compute the edge groups based on intersections between edges
    EdgeGroupVector<EdgeGroup> edge_groups;
    EdgeVector<EdgeGroupId> edge_id_to_edge_group_id(edges.size());
    EdgeVector<EdgeId> edge_id_parent(edges.size());
    EdgeVector<EdgeId> edge_id_root(edges.size());
    for (EdgeId edge_id(0); edge_id < edges.size(); ++edge_id) {
        edge_id_parent[edge_id] = edge_id;
    }
    for (auto &intersection : intersections) {
        EdgeId edge_id_1 = intersection.first;
        EdgeId edge_id_2 = intersection.second;

        edge_id_parent[edge_id_1] = edge_id_parent[edge_id_2];
    }
    // Pick a EdgeGroupId for each root edge sequence
    std::map<EdgeId, EdgeGroupId> _edge_id_to_edge_group_id_map;
    for (EdgeId edge_id(0); edge_id < edges.size(); ++edge_id) {
        // If root
        if (edge_id_parent[edge_id] == edge_id) {
            _edge_id_to_edge_group_id_map[edge_id] =
                EdgeGroupId(_edge_id_to_edge_group_id_map.size());
        }
    }
    // Assign the EdgeGroupId to each edge sequence based on its root
    // parent
    for (EdgeId edge_id(0); edge_id < edges.size(); ++edge_id) {
        // Find the root parent of the edge sequence
        std::vector<EdgeId> path({edge_id});
        while (edge_id_parent[path.back()] != path.back()) {
            path.push_back(edge_id_parent[path.back()]);
        }
        for (const EdgeId &edge_id : path) {
            edge_id_parent[edge_id] = path.back();
        }
    }
    // Build the edge sequence groups based on the mapping
    edge_groups.resize(_edge_id_to_edge_group_id_map.size());
    for (EdgeId edge_id(0); edge_id < edges.size(); ++edge_id) {
        // Get the edge sequence group id
        EdgeGroupId edge_group_id =
            _edge_id_to_edge_group_id_map[edge_id_parent[edge_id]];

        // Add the edge sequence to its group
        edge_groups[edge_group_id].edge_ids.push_back(edge_id);

        // Store the mapping from edge sequence to edge sequence group
        edge_id_to_edge_group_id[edge_id] = edge_group_id;
    }

    /* ------------------------------ Outlines
     * ------------------------------ */

    // Compute the mapping from edge sequence to outline
    EdgeVector<OutlineId> edge_id_to_outline_id(edges.size());
    for (OutlineId outline_id(0); outline_id < outlines.size(); ++outline_id) {
        const auto &outline = outlines[outline_id];
        for (const auto &polygon : outline.polygons) {
            for (const auto &edge_id : polygon) {
                edge_id_to_outline_id[edge_id] = outline_id;
            }
        }
    }

    /* ------------------------- Optimization units
     * ------------------------- */

    // Compute the optimization units based on edge sequence groups
    OptimizationUnitVector<OptimizationUnit> optim_units;
    EdgeGroupVector<OptimizationUnitId> edge_group_id_to_optim_unit_id(
        edge_groups.size());
    EdgeGroupVector<bool> edge_group_visited(edge_groups.size(), false);
    for (EdgeGroupId edge_group_id(0); edge_group_id < edge_groups.size();
         ++edge_group_id) {

        if (edge_group_visited[edge_group_id]) {
            continue;
        }

        // Get a new index for the optimization unit
        OptimizationUnitId optim_unit_id = optim_units.size_as_strong_index();

        // Find all edge sequence groups that are connected to the current
        // edge sequence group through shared outlines
        std::vector<EdgeGroupId> edge_group_ids({edge_group_id});
        std::vector<EdgeGroupId> edge_group_ids_to_visit({edge_group_id});
        while (!edge_group_ids_to_visit.empty()) {
            EdgeGroupId current_edge_group_id = edge_group_ids_to_visit.back();
            edge_group_ids_to_visit.pop_back();
            edge_group_visited[current_edge_group_id] = true;

            // Get the edge sequences in the current edge sequence group
            const auto &current_edge_group = edge_groups[current_edge_group_id];
            for (const auto &edge_seq_id : current_edge_group.edge_ids) {
                // Get the next and previous edge sequence id
                EdgeId next_edge_seq_id = next_edge_ids[edge_seq_id];
                EdgeId prev_edge_seq_id = prev_edge_ids[edge_seq_id];

                // Get the edge sequence group id of the next and previous
                // edge sequence
                EdgeGroupId next_edge_seq_group_id =
                    edge_id_to_edge_group_id[next_edge_seq_id];
                EdgeGroupId prev_edge_seq_group_id =
                    edge_id_to_edge_group_id[prev_edge_seq_id];

                // If the next edge sequence group is not visited, add it to
                // the optimization unit
                if (!edge_group_visited[next_edge_seq_group_id]) {
                    edge_group_ids_to_visit.push_back(next_edge_seq_group_id);
                    edge_group_ids.push_back(next_edge_seq_group_id);
                }
                // If the previous edge sequence group is not visited, add
                // it to the optimization unit
                if (!edge_group_visited[prev_edge_seq_group_id]) {
                    edge_group_ids_to_visit.push_back(prev_edge_seq_group_id);
                    edge_group_ids.push_back(prev_edge_seq_group_id);
                }
            }
        }

        // Build the optimization unit based on the edge sequence groups
        optim_units.push_back(OptimizationUnit(edge_group_ids));
        for (const auto &edge_seq_group_id : edge_group_ids) {
            edge_group_id_to_optim_unit_id[edge_seq_group_id] = optim_unit_id;
        }
    }

    return AllOutlines(edges, prev_edge_ids, next_edge_ids, edge_groups,
                       edge_id_to_edge_group_id, outlines,
                       edge_id_to_outline_id, optim_units,
                       edge_group_id_to_optim_unit_id);
}

double AllLines::AllOutlines::compute_metric(
    AllLines::EdgeSeqId edge_seq_id, std::vector<double> offsets,
    const PtsStructs::StoragePtr las_points, const KdTree_2 &las_kd_tree,
    std::vector<double> &metrics) {
    // TODO: Compute the metric efficiently for all offsets by projecting
    // only once and then checking whether the projected points are within
    // the edge segment for each offset

    /* ----- Compute the bounding box in the two extremes of the offsets
     * ---- */

    const std::vector<std::size_t> current_las_indices =
        las_kd_tree.search_indices_in_box(base_segment.bbox(),
                                          max_absolute_offset);

    std::vector<PtsStructs::PointId> current_las_point_ids;
    current_las_point_ids.reserve(current_las_indices.size());
    for (std::size_t idx : current_las_indices) {
        current_las_point_ids.emplace_back(PtsStructs::PointId(idx));
    }

    // Compute the weights for the LAS points
    std::vector<double> weights;
    compute_weights(las_points, current_las_point_ids, weights);

    // Compute the metric for each offset
    metrics.resize(offsets.size(), 0.0);
    for (size_t i = 0; i < offsets.size(); ++i) {
        double offset = offsets[i];
        Point_2 new_main_start =
            main_start + offset * prev_scale * prev_direction;
        Point_2 new_main_end = main_end + offset * next_scale * next_direction;

        Roofprints::Edge new_main_edge(new_main_start, new_main_end);
        if (new_main_edge.get_length() < 1e-6) {
            std::cout << "The main edge is too small, skipping metric "
                         "computation for this offset"
                      << std::endl;
            continue;
        }
        Roofprints::Edge new_prev_edge(prev_start, new_main_start);
        Roofprints::Edge new_next_edge(new_main_end, next_end);

        // Sum the scores for all points in the LAS point cloud
        double total_score_main = 0.0;
        double total_score_prev = 0.0;
        double total_score_next = 0.0;
        for (size_t j = 0; j < current_las_point_ids.size(); ++j) {
            const PtsStructs::PointId las_point_id = current_las_point_ids[j];
            const Point_2 &point = las_points->get_point_2d(las_point_id);

            double score_main = compute_metric(point, new_main_edge);
            double score_prev = compute_metric(point, new_prev_edge);
            double score_next = compute_metric(point, new_next_edge);

            total_score_main += score_main * weights[las_point_id];
            total_score_prev += score_prev * weights[las_point_id];
            total_score_next += score_next * weights[las_point_id];
        }

        double length_penalization_main =
            std::min(new_main_edge.get_length(), MAX_LENGTH_PENALIZATION);
        double length_penalization_prev =
            std::min(new_prev_edge.get_length(), MAX_LENGTH_PENALIZATION);
        double length_penalization_next =
            std::min(new_next_edge.get_length(), MAX_LENGTH_PENALIZATION);

        metrics[i] = (total_score_main / length_penalization_main) +
                     (total_score_prev / length_penalization_prev) +
                     (total_score_next / length_penalization_next);
    }
}

double AllLines::AllOutlines::compute_optimal_offset(
    EdgeGroupId edge_seq_group_id, double max_absolute_offset,
    double offset_step, const KdTree_2 &las_kd_tree,
    PtsStructs::StoragePtr las_points) {
    // auto [min_offset_limit, max_offset_limit] =
    // _compute_movement_limits();
    auto [min_offset_limit, max_offset_limit] =
        std::make_pair(-std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::infinity());
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
            moving_direction, offsets, all_points, las_kd_tree, las_points,
            metrics);
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

    return best_offset;
}

void AllLines::AllOutlines::optimize_unit(
    const PtsStructs::StoragePtr las_points,
    const OptimizationUnitId &optim_unit_id) {
    const auto &optim_unit = optim_units[optim_unit_id];
    std::cout << "Optimizing unit " << optim_unit_id << " with "
              << optim_unit.edge_seq_group_ids.size() << " edge sequence groups"
              << std::endl;
}

void AllLines::AllOutlines::optimize_all_units(
    const PtsStructs::StoragePtr las_points) {
    for (OptimizationUnitId optim_unit_id(0);
         optim_unit_id < optim_units.size(); ++optim_unit_id) {
        optimize_unit(las_points, optim_unit_id);
    }
}

void AllLines::compute_roofprints(
    const std::string &input_las_file,
    const std::string &input_bd_topo_edges_file,
    const std::string &input_bd_topo_intersections_file,
    const std::string &output_roofprints_file, bool overwrite) {

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
    std::vector<BDTOPOEdge> initial_edges;
    std::vector<std::pair<uint32_t, uint32_t>> _intersections;
    auto status = read_bd_topo_as_grouped_edges(
        input_bd_topo_edges_file, input_bd_topo_intersections_file,
        initial_edges, _intersections);
    if (!status.ok()) {
        std::cerr << "Error reading BD TOPO: " << status.ToString()
                  << std::endl;
        throw std::runtime_error("Failed to read edges from BD TOPO");
    }
    std::cout << "Successfully read " << initial_edges.size()
              << " edges from BD TOPO." << std::endl;

    // Format the edges for the AllOutlines data structures
    EdgeVector<Edge> edges;
    std::map<uint32_t, EdgeId> edge_key_map;
    std::vector<std::pair<EdgeId, EdgeId>> intersections;
    for (const auto &edge : initial_edges) {
        edges.push_back(Edge(edge.start, edge.end));
        edge_key_map[edge.edge_key] = EdgeId(edges.size() - 1);
    }
    for (const auto &intersection : _intersections) {
        uint32_t edge_key_1 = intersection.first;
        uint32_t edge_key_2 = intersection.second;
        intersections.push_back(
            {edge_key_map[edge_key_1], edge_key_map[edge_key_2]});
    }

    // Rebuild the MultiPolygon hierarchy based on the edges and their
    // building, polygon, and ring indices
    std::map<std::string, std::vector<std::vector<EdgeId>>>
        building_id_to_polygons;
    std::map<std::string, std::vector<std::vector<bool>>> found_edges;
    for (const auto &edge : initial_edges) {
        auto building_id = edge.building_id;
        auto polygon_idx = edge.polygon_idx;
        auto ring_idx = edge.ring_idx;
        auto edge_id = edge.edge_key;

        if (building_id_to_polygons[building_id].size() <= polygon_idx) {
            building_id_to_polygons[building_id].resize(polygon_idx + 1);
            found_edges[building_id].resize(polygon_idx + 1);
        }
        if (building_id_to_polygons[building_id][polygon_idx].size() <=
            ring_idx) {
            building_id_to_polygons[building_id][polygon_idx].resize(ring_idx +
                                                                     1);
            found_edges[building_id][polygon_idx].resize(ring_idx + 1, false);
        }
        building_id_to_polygons[building_id][polygon_idx][ring_idx] =
            edge_key_map[edge_id];
        found_edges[building_id][polygon_idx][ring_idx] = true;
    }

    // Check for any missing edges in the hierarchy and throw an error if
    // any are found
    for (const auto &[building_id, polygons] : building_id_to_polygons) {
        for (std::size_t polygon_idx = 0; polygon_idx < polygons.size();
             ++polygon_idx) {
            for (std::size_t ring_idx = 0;
                 ring_idx < polygons[polygon_idx].size(); ++ring_idx) {
                if (!found_edges[building_id][polygon_idx][ring_idx]) {
                    throw std::runtime_error(
                        "Warning: Missing edge for building " + building_id +
                        ", polygon " + std::to_string(polygon_idx) + ", ring " +
                        std::to_string(ring_idx));
                }
            }
        }
    }

    // Build the expected structure
    OutlineVector<OutlineAsEdges> outlines;
    for (const auto &[building_id, polygons] : building_id_to_polygons) {
        std::vector<std::vector<EdgeId>> outline_polygons;
        for (const auto &polygon : polygons) {
            std::vector<EdgeId> outline_edges;
            for (const auto &edge_id : polygon) {
                outline_edges.push_back(edge_id);
            }
            if (!outline_edges.empty()) {
                outline_polygons.push_back(outline_edges);
            } else {
                std::cerr << "Warning: Polygon with no edges in building "
                          << building_id
                          << ", skipping this polygon in the outline."
                          << std::endl;
            }
        }
        if (!outline_polygons.empty()) {
            outlines.push_back(OutlineAsEdges(outline_polygons));
        } else {
            std::cerr << "Warning: Building " << building_id
                      << " has no valid polygons, skipping this building in "
                         "the outlines."
                      << std::endl;
        }
    }

    // Build the AllOutlines data structure
    AllOutlines all_outlines =
        make_all_outlines(edges, outlines, intersections);

    // Optimize the outlines
    all_outlines.optimize_all_units(las_reader.points);

    // // Write the roofprints to a Parquet file
    // std::cout << "Writing roofprints to Parquet file..." << std::endl;
    // auto write_status = write_geoms_to_parquet(
    //     optimized_edges, output_roofprints_file, overwrite);

    // if (!write_status.ok()) {
    //     std::cerr << "Error writing roofprints to Parquet: "
    //               << write_status.ToString() << std::endl;
    //     throw std::runtime_error("Failed to write roofprints to
    //     Parquet");
    // }
}