#include "topology.hpp"

#include <boost/property_map/property_map.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "../parquet.hpp"
#include "../points.hpp"
#include "../utils/cgal.hpp"
#include "../utils/pbar.hpp"
#include "constants.hpp"
#include "criterion.hpp"
#include "line_mover.hpp"

namespace {

Bbox_2 edge_bbox_buffered(EdgeMatching::Edge focus_edge,
                          EdgeMatching::Edge prev_edge,
                          EdgeMatching::Edge next_edge, double buffer_normal,
                          double buffer_tangent) {
    Point_2 start =
        CustomCGAL::intersection(focus_edge.to_line(), prev_edge.to_line());
    Point_2 end =
        CustomCGAL::intersection(focus_edge.to_line(), next_edge.to_line());
    Segment_2 segment(start, end);
    Bbox_2 bbox = segment.bbox();
    Vector_2 normal = focus_edge.get_normal() * buffer_normal;
    Vector_2 tangent = focus_edge.get_direction() * buffer_tangent;
    double x_buffer = std::abs(normal.x()) + std::abs(tangent.x());
    double y_buffer = std::abs(normal.y()) + std::abs(tangent.y());
    return Bbox_2(bbox.xmin() - x_buffer, bbox.ymin() - y_buffer,
                  bbox.xmax() + x_buffer, bbox.ymax() + y_buffer);
}

} // namespace

EdgeMatching::Edge::Edge(Point_3 initial_start, Point_3 initial_end,
                         uint32_t key)
    : initial_start(initial_start), initial_end(initial_end), key(key) {
    Point_2 start_2d(initial_start.x(), initial_start.y());
    Point_2 end_2d(initial_end.x(), initial_end.y());
    line = Line_2(start_2d, end_2d);
    direction = UnitVector_2(line.direction().to_vector());
}

Point_3 EdgeMatching::Edge::get_initial_start() const { return initial_start; }
Point_3 EdgeMatching::Edge::get_initial_end() const { return initial_end; }
UnitVector_2 EdgeMatching::Edge::get_direction() const { return direction; }
UnitVector_2 EdgeMatching::Edge::get_normal() const {
    return direction.perpendicular(CGAL::CLOCKWISE);
}
Line_2 EdgeMatching::Edge::to_line() const { return line; }

void EdgeMatching::Edge::translate(Vector_2 offset) {
    *this = translated(offset);
}

EdgeMatching::Edge EdgeMatching::Edge::translated(Vector_2 offset) const {
    Vector_3 offset_3(offset.x(), offset.y(), 0.00);
    Point_3 new_start = initial_start + offset_3;
    Point_3 new_end = initial_end + offset_3;
    return Edge(new_start, new_end, key);
}

EdgeMatching::OutlineAsEdges::OutlineAsEdges(
    const std::vector<std::vector<std::vector<EdgeId>>> &multi_polygons)
    : multi_polygons(multi_polygons) {}

EdgeMatching::OptimizationUnit::OptimizationUnit(
    const std::vector<EdgeGroupId> &edge_group_ids)
    : edge_group_ids(edge_group_ids) {}

EdgeMatching::AllOutlines::AllOutlines(
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

const EdgeMatching::Edge &
EdgeMatching::AllOutlines::get_edge(EdgeId edge_id) const {
    return edges.at(edge_id);
}

const EdgeMatching::EdgeGroup &
EdgeMatching::AllOutlines::get_edge_group(EdgeGroupId edge_group_id) const {
    return edge_groups.at(edge_group_id);
}

const EdgeMatching::OutlineAsEdges &
EdgeMatching::AllOutlines::get_outline(OutlineId outline_id) const {
    return outlines.at(outline_id);
}

EdgeMatching::EdgeGroupId
EdgeMatching::AllOutlines::get_edge_group_id(EdgeId edge_id) const {
    return edge_id_to_edge_group_id.at(edge_id);
}

EdgeMatching::OutlineId
EdgeMatching::AllOutlines::get_outline_id(EdgeId edge_id) const {
    return edge_id_to_outline_id.at(edge_id);
}

EdgeMatching::OptimizationUnitId
EdgeMatching::AllOutlines::get_optim_unit_id(EdgeGroupId edge_group_id) const {
    return edge_group_id_to_optim_unit_id.at(edge_group_id);
}

EdgeMatching::EdgeId
EdgeMatching::AllOutlines::get_prev_edge_id(EdgeId edge_id) const {
    return prev_edge_ids.at(edge_id);
}

EdgeMatching::EdgeId
EdgeMatching::AllOutlines::get_next_edge_id(EdgeId edge_id) const {
    return next_edge_ids.at(edge_id);
}

Point_2 EdgeMatching::AllOutlines::get_edge_start(EdgeId edge_id) const {
    EdgeId prev_edge_id = get_prev_edge_id(edge_id);
    Edge current_edge = get_edge(edge_id);
    Edge prev_edge = get_edge(prev_edge_id);
    return CustomCGAL::intersection(current_edge.to_line(),
                                    prev_edge.to_line());
}

Point_2 EdgeMatching::AllOutlines::get_edge_end(EdgeId edge_id) const {
    EdgeId next_edge_id = get_next_edge_id(edge_id);
    Edge current_edge = get_edge(edge_id);
    Edge next_edge = get_edge(next_edge_id);
    return CustomCGAL::intersection(current_edge.to_line(),
                                    next_edge.to_line());
}

EdgeMatching::EdgeId EdgeMatching::AllOutlines::edge_count() const {
    return edges.size_as_strong_index();
}

EdgeMatching::EdgeGroupId EdgeMatching::AllOutlines::edge_group_count() const {
    return edge_groups.size_as_strong_index();
}

EdgeMatching::OutlineId EdgeMatching::AllOutlines::outline_count() const {
    return outlines.size_as_strong_index();
}

EdgeMatching::OptimizationUnitId
EdgeMatching::AllOutlines::optim_unit_count() const {
    return optim_units.size_as_strong_index();
}

EdgeMatching::AllOutlines::AllOutlines(
    const EdgeVector<Edge> &edges,
    const OutlineVector<OutlineAsEdges> &outlines,
    const std::vector<std::pair<EdgeId, EdgeId>> &intersections) {

    /* -------------------------------- Edges
     * ------------------------------- */

    // Find the previous and next edge for each edge
    EdgeVector<EdgeId> prev_edge_ids(edges.size());
    EdgeVector<EdgeId> next_edge_ids(edges.size());
    // for (const auto &outline : outlines) {
    //     if (outline.polygons.empty()) {
    //         throw std::runtime_error("Outline has no polygons");
    //     }
    //     for (const auto &polygon : outline.polygons) {
    //         if (polygon.empty()) {
    //             throw std::runtime_error("Polygon has no edges");
    //         }
    //         for (std::size_t i = 0; i < polygon.size(); ++i) {
    //             EdgeId edge_id = polygon[i];
    //             prev_edge_ids[edge_id] =
    //                 polygon[(i - 1 + polygon.size()) % polygon.size()];
    //             next_edge_ids[edge_id] = polygon[(i + 1) %
    //             polygon.size()];
    //         }
    //     }
    // }

    for (OutlineId outline_id(0); outline_id < outlines.size(); ++outline_id) {
        const auto &outline = outlines[outline_id];
        if (outline.multi_polygons.empty()) {
            throw std::runtime_error("Outline has no multi polygons");
        }
        for (std::size_t multi_polygon_id = 0;
             multi_polygon_id < outline.multi_polygons.size();
             multi_polygon_id++) {
            for (std::size_t polygon_id = 0;
                 polygon_id < outline.multi_polygons[multi_polygon_id].size();
                 ++polygon_id) {
                const auto &polygon =
                    outline.multi_polygons[multi_polygon_id][polygon_id];
                if (polygon.empty()) {
                    throw std::runtime_error("Polygon has no edges");
                }
                for (std::size_t edge_id_index = 0;
                     edge_id_index < polygon.size(); ++edge_id_index) {
                    EdgeId edge_id = polygon[edge_id_index];

                    prev_edge_ids[edge_id] =
                        polygon[(edge_id_index - 1 + polygon.size()) %
                                polygon.size()];
                    next_edge_ids[edge_id] =
                        polygon[(edge_id_index + 1) % polygon.size()];
                }
            }
        }
    }

    std::cout << "Found previous and next edges for each edge" << std::endl;

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
        for (const auto &multi_polygon : outline.multi_polygons) {
            for (const auto &polygon : multi_polygon) {
                for (const auto &edge_id : polygon) {
                    edge_id_to_outline_id[edge_id] = outline_id;
                }
            }
        }
    }

    /* ------------------------- Optimization units
     * ------------------------- */

    // Compute the optimization units based on edge groups
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

        // Find all edge groups that are connected to the current edge group
        // through shared outlines
        std::set<EdgeGroupId> edge_group_ids({edge_group_id});
        std::vector<EdgeGroupId> edge_group_ids_to_visit({edge_group_id});
        while (!edge_group_ids_to_visit.empty()) {
            EdgeGroupId current_edge_group_id = edge_group_ids_to_visit.back();
            edge_group_ids_to_visit.pop_back();
            edge_group_visited[current_edge_group_id] = true;

            // Get the edges in the current edge group
            const auto &current_edge_group = edge_groups[current_edge_group_id];
            for (const auto &edge_seq_id : current_edge_group.edge_ids) {
                // Get the next and previous edge id
                EdgeId next_edge_seq_id = next_edge_ids[edge_seq_id];
                EdgeId prev_edge_seq_id = prev_edge_ids[edge_seq_id];

                // Get the edge group id of the next and previous edge
                EdgeGroupId next_edge_seq_group_id =
                    edge_id_to_edge_group_id[next_edge_seq_id];
                EdgeGroupId prev_edge_seq_group_id =
                    edge_id_to_edge_group_id[prev_edge_seq_id];

                // If the next edge group is not visited, add it to the
                // optimization unit
                if (!edge_group_visited[next_edge_seq_group_id]) {
                    edge_group_ids_to_visit.push_back(next_edge_seq_group_id);
                    edge_group_ids.insert(next_edge_seq_group_id);
                }
                // If the previous edge group is not visited, add it to the
                // optimization unit
                if (!edge_group_visited[prev_edge_seq_group_id]) {
                    edge_group_ids_to_visit.push_back(prev_edge_seq_group_id);
                    edge_group_ids.insert(prev_edge_seq_group_id);
                }
            }
        }

        // Build the optimization unit based on the edge groups
        optim_units.push_back(OptimizationUnit(std::vector<EdgeGroupId>(
            edge_group_ids.begin(), edge_group_ids.end())));
        for (const auto &edge_seq_group_id : edge_group_ids) {
            edge_group_id_to_optim_unit_id[edge_seq_group_id] = optim_unit_id;
        }
    }

    this->edges = edges;
    this->prev_edge_ids = prev_edge_ids;
    this->next_edge_ids = next_edge_ids;
    this->edge_groups = edge_groups;
    this->edge_id_to_edge_group_id = edge_id_to_edge_group_id;
    this->outlines = outlines;
    this->edge_id_to_outline_id = edge_id_to_outline_id;
    this->optim_units = optim_units;
    this->edge_group_id_to_optim_unit_id = edge_group_id_to_optim_unit_id;
}

void EdgeMatching::AllOutlines::compute_weights(
    PtsStructs::StoragePtr las_points,
    const std::vector<PtsStructs::PointId> &point_ids,
    std::vector<double> &weights,
    std::vector<Vector_2> &point_inward_dirs) const {
    (void)las_points;
    (void)point_ids;
    (void)weights;
    (void)point_inward_dirs;
    throw std::logic_error(
        "AllOutlines::compute_weights must be overridden in subclasses");
}

void EdgeMatching::AllOutlines::prepare_offsets(
    std::vector<double> &offsets) const {
    (void)offsets;
    throw std::logic_error(
        "AllOutlines::prepare_offsets must be overridden in subclasses");
}

std::unique_ptr<ICriterion> EdgeMatching::AllOutlines::create_criterion(
    std::vector<Point_2> points, std::vector<double> weights,
    std::vector<Vector_2> point_inward_dirs,
    std::vector<LASclassification::Value> classifications) const {
    // Default implementation throws; subclasses must override
    (void)points;
    (void)weights;
    (void)point_inward_dirs;
    (void)classifications;
    throw std::logic_error(
        "AllOutlines::create_criterion must be overridden in subclasses");
}

void EdgeMatching::AllOutlines::get_bbox_configs(
    const std::vector<std::map<EdgeId, double>> &configs,
    const std::set<EdgeMatching::EdgeId> edge_ids,
    const UnitVector_2 offset_direction, Bbox_2 &bbox) const {
    Bbox_2 all_cases_bbox;
    for (const auto &config : configs) {
        for (const auto &edge_id : edge_ids) {
            double shift = 0.0;
            if (config.find(edge_id) != config.end()) {
                shift = config.at(edge_id);
            }
            Edge focus_edge =
                get_edge(edge_id).translated(shift * offset_direction);

            EdgeId prev_edge_id = get_prev_edge_id(edge_id);
            Edge prev_edge = get_edge(prev_edge_id);
            if (config.find(prev_edge_id) != config.end()) {
                prev_edge = prev_edge.translated(config.at(prev_edge_id) *
                                                 offset_direction);
            }

            EdgeId next_edge_id = get_next_edge_id(edge_id);
            Edge next_edge = get_edge(next_edge_id);
            if (config.find(next_edge_id) != config.end()) {
                next_edge = next_edge.translated(config.at(next_edge_id) *
                                                 offset_direction);
            }

            Bbox_2 edge_bbox =
                edge_bbox_buffered(focus_edge, prev_edge, next_edge,
                                   EDGE_CRITERION_MAX_DISTANCE, 0.0);
            all_cases_bbox += edge_bbox;
        }
    }
    bbox = all_cases_bbox;
}

void EdgeMatching::AllOutlines::find_relevant_points(
    const std::vector<std::map<EdgeId, double>> &configs,
    const std::set<EdgeMatching::EdgeId> edge_ids,
    const UnitVector_2 offset_direction,
    const PtsStructs::StoragePtr las_points,
    std::vector<std::size_t> &las_indices) const {
    (void)configs;
    (void)edge_ids;
    (void)offset_direction;
    (void)las_points;
    (void)las_indices;
    throw std::logic_error("AllOutlines::find_relevant_points must be "
                           "overridden in subclasses");
}

void EdgeMatching::AllOutlines::compute_metrics(
    EdgeMatching::EdgeGroupId edge_group_id, std::vector<double> offsets,
    UnitVector_2 offset_direction, const PtsStructs::StoragePtr las_points,
    std::vector<double> &metrics,
    std::vector<std::map<EdgeMatching::EdgeId, double>> &configs) const {
    // TODO: Compute the metric efficiently for all offsets by projecting
    // only once and then checking whether the projected points are within
    // the edge segment for each offset

    // Get the edge group
    EdgeGroup edge_group = get_edge_group(edge_group_id);
    if (edge_group.edge_ids.empty()) {
        throw std::runtime_error("Edge group has no edges");
    }

    // Separate the positive and negative offsets and sort them by absolute
    // value. Also keep the original indices of the offsets to be able to
    // return the metrics in the correct order
    std::vector<size_t> pos_offset_indices;
    std::vector<size_t> negative_offset_indices;
    for (size_t i = 0; i < offsets.size(); ++i) {
        if (offsets[i] >= 0) {
            pos_offset_indices.push_back(i);
        } else {
            negative_offset_indices.push_back(i);
        }
    }
    std::sort(pos_offset_indices.begin(), pos_offset_indices.end(),
              [&offsets](size_t a, size_t b) {
                  return std::abs(offsets[a]) < std::abs(offsets[b]);
              });
    std::sort(negative_offset_indices.begin(), negative_offset_indices.end(),
              [&offsets](size_t a, size_t b) {
                  return std::abs(-offsets[a]) < std::abs(-offsets[b]);
              });
    std::vector<double> pos_offsets;
    for (const auto &idx : pos_offset_indices) {
        pos_offsets.push_back(offsets[idx]);
    }
    std::vector<double> neg_offsets;
    for (const auto &idx : negative_offset_indices) {
        neg_offsets.push_back(-offsets[idx]);
    }

    // Compute the configurations of the edge and its
    // neighbours for all offsets
    std::vector<std::map<EdgeMatching::EdgeId, double>> pos_configurations;
    std::vector<std::map<EdgeMatching::EdgeId, double>> neg_configurations;
    LineMoverSimple pos_line_mover(*this, edge_group_id, offset_direction,
                                   pos_offsets);
    LineMoverSimple neg_line_mover(*this, edge_group_id, -offset_direction,
                                   neg_offsets);
    pos_line_mover.compute_all();
    neg_line_mover.compute_all();
    pos_line_mover.get_computed_shifts(pos_configurations);
    neg_line_mover.get_computed_shifts(neg_configurations);

    // Gather offsets and configurations together
    std::vector<double> pos_neg_offsets = pos_offsets;
    for (double neg_offset : neg_offsets) {
        pos_neg_offsets.push_back(-neg_offset);
    }
    std::vector<std::map<EdgeMatching::EdgeId, double>> pos_neg_configurations =
        pos_configurations;
    for (const auto &neg_config : neg_configurations) {
        std::map<EdgeMatching::EdgeId, double> pos_neg_config;
        for (const auto &[edge_id, offset] : neg_config) {
            pos_neg_config[edge_id] = -offset;
        }
        pos_neg_configurations.push_back(pos_neg_config);
    }
    std::vector<size_t> pos_neg_offset_indices = pos_offset_indices;
    pos_neg_offset_indices.insert(pos_neg_offset_indices.end(),
                                  negative_offset_indices.begin(),
                                  negative_offset_indices.end());

    // Gather the edges that are shifted in any of the configurations
    std::set<EdgeMatching::EdgeId> _encountered_edge_ids;
    for (const auto &config : pos_neg_configurations) {
        for (const auto &pair : config) {
            _encountered_edge_ids.insert(pair.first);
        }
    }

    // Make sure all the neighbours of the encountered edges are also
    // included
    std::set<EdgeMatching::EdgeId> criterion_edge_ids(_encountered_edge_ids);
    for (const auto &edge_id : _encountered_edge_ids) {
        criterion_edge_ids.insert(get_prev_edge_id(edge_id));
        criterion_edge_ids.insert(get_next_edge_id(edge_id));
    }

    // Find the relevant LAS points for the metric computation based on the
    // configurations of the edges
    std::vector<std::size_t> current_las_indices;
    find_relevant_points(pos_neg_configurations, criterion_edge_ids,
                         offset_direction, las_points, current_las_indices);

    std::vector<PtsStructs::PointId> current_las_point_ids;
    current_las_point_ids.reserve(current_las_indices.size());
    for (std::size_t idx : current_las_indices) {
        current_las_point_ids.emplace_back(PtsStructs::PointId(idx));
    }

    // Compute the weights for the LAS points
    std::vector<double> weights;
    std::vector<Vector_2> point_inward_dirs;
    compute_weights(las_points, current_las_point_ids, weights,
                    point_inward_dirs);

    // Prepare the class to compute the criterion
    std::vector<Point_2> selected_las_points(current_las_point_ids.size());
    std::vector<LASclassification::Value> selected_las_classification(
        current_las_point_ids.size());
    for (size_t i = 0; i < current_las_point_ids.size(); ++i) {
        const auto &point_id = current_las_point_ids[i];
        selected_las_points[i] = las_points->get_point_2d(point_id);
        selected_las_classification[i] =
            las_points->get_classification(point_id);
    }
    std::unique_ptr<ICriterion> criterion = create_criterion(
        std::move(selected_las_points), std::move(weights),
        std::move(point_inward_dirs), std::move(selected_las_classification));

    // Compute the metric for each offset
    metrics.resize(offsets.size(), 0.0);
    configs.resize(offsets.size());
    for (std::size_t i = 0; i < pos_neg_offsets.size(); ++i) {
        double offset = pos_neg_offsets[i];
        const auto &config = pos_neg_configurations[i];

        // Build the segments for the current configuration
        std::map<EdgeMatching::EdgeId, Edge> current_edges;
        for (EdgeId edge_id : criterion_edge_ids) {
            double shift = 0.0;
            if (config.find(edge_id) != config.end()) {
                shift = config.at(edge_id);
            }
            const Edge &edge = get_edge(edge_id);
            current_edges.emplace(edge_id,
                                  edge.translated(shift * offset_direction));
        }

        std::vector<Segment_2> segments;
        std::vector<double> segments_initial_length;
        std::vector<UnitVector_2> segments_inward_normals;
        segments.reserve(criterion_edge_ids.size());
        segments_initial_length.reserve(criterion_edge_ids.size());
        segments_inward_normals.reserve(criterion_edge_ids.size());

        for (EdgeId edge_id : criterion_edge_ids) {
            Edge edge = current_edges.at(edge_id);
            EdgeId prev_edge_id = get_prev_edge_id(edge_id);
            Edge prev_edge = get_edge(prev_edge_id);
            if (current_edges.find(prev_edge_id) != current_edges.end()) {
                prev_edge = current_edges.at(prev_edge_id);
            }
            Point_2 start =
                CustomCGAL::intersection(edge.to_line(), prev_edge.to_line());

            EdgeId next_edge_id = get_next_edge_id(edge_id);
            Edge next_edge = get_edge(next_edge_id);
            if (current_edges.find(next_edge_id) != current_edges.end()) {
                next_edge = current_edges.at(next_edge_id);
            }
            Point_2 end =
                CustomCGAL::intersection(edge.to_line(), next_edge.to_line());

            segments.push_back(Segment_2(start, end));
            Vector_3 initial_start_to_end =
                edge.get_initial_end() - edge.get_initial_start();

            Vector_2 initial_start_to_end_2d(initial_start_to_end.x(),
                                             initial_start_to_end.y());
            segments_initial_length.push_back(
                std::sqrt(initial_start_to_end_2d.squared_length()));

            Vector_2 inward_normal =
                initial_start_to_end_2d.perpendicular(CGAL::COUNTERCLOCKWISE);
            segments_inward_normals.push_back(inward_normal);
        }

        // Compute the metric for the current configuration
        double metric = criterion->evaluate_segments(
            segments, segments_initial_length, segments_inward_normals);
        metrics.at(pos_neg_offset_indices[i]) = metric;
        configs.at(pos_neg_offset_indices[i]) = config;
    }
}

void EdgeMatching::AllOutlines::compute_optimal_offset(
    EdgeGroupId edge_group_id, UnitVector_2 offset_direction,
    PtsStructs::StoragePtr las_points, double &best_offset,
    std::map<EdgeMatching::EdgeId, double> &best_config) const {
    // Prepare the offsets to evaluate
    std::vector<double> offsets;
    prepare_offsets(offsets);

    // Compute the metrics for all offsets
    std::vector<double> metrics;
    std::vector<std::map<EdgeMatching::EdgeId, double>> configs;
    compute_metrics(edge_group_id, offsets, offset_direction, las_points,
                    metrics, configs);

    // Find the best offset
    size_t best_offset_index = -1;
    double best_metric = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < offsets.size(); ++i) {
        // Add a small penalty to the metric based on the absolute value of
        // the offset to prefer smaller offsets in case of ties
        double metric = metrics[i] + std::abs(offsets[i]) * 1e-6;
        if (metric < best_metric) {
            best_metric = metric;
            best_offset_index = i;
        }
    }

    if (best_offset_index == static_cast<size_t>(-1)) {
        throw std::runtime_error("No valid offset found");
    }
    best_offset = offsets[best_offset_index];
    best_config = configs[best_offset_index];
}

void EdgeMatching::AllOutlines::optimize_unit(
    const PtsStructs::StoragePtr las_points,
    const OptimizationUnitId &optim_unit_id) {
    const auto &optim_unit = optim_units[optim_unit_id];
    // Compute the total length of each edge group in the optimization unit
    // to prioritize longer edge groups in the optimization
    std::vector<std::pair<EdgeGroupId, double>> edge_groups_with_length;
    for (const auto &edge_group_id : optim_unit.edge_group_ids) {
        const auto &edge_group = get_edge_group(edge_group_id);
        double total_length = 0.0;
        for (const auto &edge_id : edge_group.edge_ids) {
            total_length +=
                std::sqrt((get_edge_end(edge_id) - get_edge_start(edge_id))
                              .squared_length());
        }
        edge_groups_with_length.emplace_back(edge_group_id, total_length);
    }

    // Sort edge groups by length in descending order
    std::sort(edge_groups_with_length.begin(), edge_groups_with_length.end(),
              [](const std::pair<EdgeGroupId, double> &a,
                 const std::pair<EdgeGroupId, double> &b) {
                  return a.second > b.second;
              });

    // Optimize each edge group in the optimization unit sequentially
    for (const auto &[edge_group_id, length] : edge_groups_with_length) {
        // Compute the offset direction based on the edge group
        EdgeGroup edge_group = get_edge_group(edge_group_id);
        if (edge_group.edge_ids.empty()) {
            throw std::runtime_error("Edge group has no edges");
        }
        EdgeId edge_id = edge_group.edge_ids[0];
        Edge edge = get_edge(edge_id);
        UnitVector_2 offset_direction = edge.get_normal();

        double best_offset;
        std::map<EdgeMatching::EdgeId, double> best_config;
        compute_optimal_offset(edge_group_id, offset_direction, las_points,
                               best_offset, best_config);

        // Apply the best configuration to the edges in the optimization
        // unit
        for (const auto &[edge_id, offset] : best_config) {
            edges[edge_id] =
                edges[edge_id].translated(offset * offset_direction);
        }
    }
}

void EdgeMatching::AllOutlines::optimize_all_units(
    const PtsStructs::StoragePtr las_points) {
    ProgressBarTotal progress_bar(optim_units.size(), "Optimizing edge groups");
    progress_bar.start();
    for (OptimizationUnitId optim_unit_id(0);
         optim_unit_id < optim_units.size(); ++optim_unit_id) {
        try {
            optimize_unit(las_points, optim_unit_id);
        } catch (const std::exception &e) {
            std::cerr << "Error optimizing unit " << optim_unit_id << ": "
                      << e.what() << std::endl;
        }
        progress_bar.increment(1);
    }
    progress_bar.finish();
}

void EdgeMatching::AllOutlines::get_multipolygons(
    const OutlineVector<OutlineAsEdges> &outline_as_edges,
    OutlineVector<MultiPolygonZ> &multi_polygons) const {

    multi_polygons.clear();
    multi_polygons.reserve(outline_as_edges.size());
    for (OutlineId i(0); i < outline_as_edges.size(); ++i) {
        OutlineAsEdges outline = outline_as_edges[i];
        std::vector<PolygonZ> polygons;

        try {
            for (const auto &multi_polygon : outline.multi_polygons) {
                std::vector<std::vector<Point_3>> rings;
                for (const auto &polygon : multi_polygon) {
                    std::vector<Point_3> ring;
                    for (const auto &edge_id : polygon) {
                        Edge edge = get_edge(edge_id);
                        Point_2 start_2d = get_edge_start(edge_id);
                        double z_start = edge.get_initial_start().z();
                        ring.push_back(
                            Point_3(start_2d.x(), start_2d.y(), z_start));
                    }
                    rings.push_back(ring);
                }
                polygons.emplace_back(PolygonZ(rings, false));
            }
        } catch (const std::exception &e) {
            std::cerr << "Error converting outline " << i
                      << " to multipolygon: " << e.what() << std::endl;
            polygons.clear();
        }
        multi_polygons.emplace_back(MultiPolygonZ(polygons));
    }
}

arrow::Status EdgeMatching::AllOutlines::export_outlines(
    const OutlineVector<OutlineAsEdges> &outlines,
    const OutlineVector<std::string> &building_ids,
    const std::string &output_footprints_file, bool overwrite) const {
    // Write the footprints to a Parquet file
    std::cout << "Writing footprints to Parquet file..." << std::endl;
    OutlineVector<MultiPolygonZ> optimized_outlines;
    get_multipolygons(outlines, optimized_outlines);
    std::vector<MultiPolygonZWithAttributes> optimized_outlines_with_attributes;
    for (OutlineId i(0); i < optimized_outlines.size(); ++i) {
        optimized_outlines_with_attributes.push_back(
            {optimized_outlines[i], building_ids[i],
             OutlineSource::Id::Unknown});
    }

    return write_geoms_to_parquet(optimized_outlines_with_attributes,
                                  output_footprints_file, overwrite);
}