#include "topology.hpp"

#include <boost/property_map/property_map.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "../las/reader.hpp"
#include "../parquet.hpp"
#include "../points.hpp"
#include "../utils/cgal.hpp"
#include "../utils/pbar.hpp"
#include "constants.hpp"
#include "criterion.hpp"
#include "line_mover.hpp"

namespace {

void compute_weights(PtsStructs::StoragePtr las_points,
                     const std::vector<PtsStructs::PointId> &point_ids,
                     std::vector<double> &weights,
                     std::vector<Vector_2> &point_inner_dirs) {
    weights.clear();
    weights.resize(point_ids.size());
    point_inner_dirs.clear();
    point_inner_dirs.resize(point_ids.size());

    // Compute the minimum and maximum Z values
    double min_z = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    for (const auto &point_id : point_ids) {
        double z =
            las_points->get_field_as<double>(pdal::Dimension::Id::Z, point_id);
        if (z < min_z) {
            min_z = z;
        }
        if (z > max_z) {
            max_z = z;
        }
    }

    // Compute the weights for each point
    for (std::size_t i = 0; i < point_ids.size(); ++i) {
        const auto &point_id = point_ids.at(i);

        // Give more weight to higher points
        double z =
            las_points->get_field_as<double>(pdal::Dimension::Id::Z, point_id);
        double height_norm = (z - min_z + 1e-6) / (max_z - min_z + 1e-6);
        // double height_factor = height_norm;
        double height_factor = 1.0;

        // Give more weight to non-generated points
        uint8_t is_generated = las_points->get_field_as<uint8_t>(
            CustomDimensions::Id::IsGenerated, point_id);
        double generated_factor = (2.0 - is_generated) / 2.0;

        // Give more weight to points classified as building
        const auto cls_raw = las_points->get_field_as<
            std::underlying_type_t<LASclassification::Value>>(
            pdal::Dimension::Id::Classification, point_id);
        const auto cls = static_cast<LASclassification::Value>(cls_raw);
        double class_factor = 0.3;
        if (cls == LASclassification::Value::Building) {
            class_factor = 1.0;
        }

        // Combine the factors to get the final weight
        weights.at(i) = height_factor * generated_factor * class_factor;

        // Extracts the normal vector for the point
        double inner_x = las_points->get_field_as<double>(
            CustomDimensions::Id::InwardVectorX, point_id);
        double inner_y = las_points->get_field_as<double>(
            CustomDimensions::Id::InwardVectorY, point_id);
        point_inner_dirs.at(i) = Vector_2(inner_x, inner_y);
    }
}

Bbox_2 edge_bbox_buffered(AllLines::Edge focus_edge, AllLines::Edge prev_edge,
                          AllLines::Edge next_edge, double buffer_normal,
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

AllLines::Edge::Edge(Point_3 initial_start, Point_3 initial_end, uint32_t key)
    : initial_start(initial_start), initial_end(initial_end), key(key) {
    Point_2 start_2d(initial_start.x(), initial_start.y());
    Point_2 end_2d(initial_end.x(), initial_end.y());
    line = Line_2(start_2d, end_2d);
    direction = UnitVector_2(line.direction().to_vector());
}

Point_3 AllLines::Edge::get_initial_start() const { return initial_start; }
Point_3 AllLines::Edge::get_initial_end() const { return initial_end; }
UnitVector_2 AllLines::Edge::get_direction() const { return direction; }
UnitVector_2 AllLines::Edge::get_normal() const {
    return direction.perpendicular(CGAL::CLOCKWISE);
}
Line_2 AllLines::Edge::to_line() const { return line; }

void AllLines::Edge::translate(Vector_2 offset) { *this = translated(offset); }

AllLines::Edge AllLines::Edge::translated(Vector_2 offset) const {
    Vector_3 offset_3(offset.x(), offset.y(), 0.00);
    Point_3 new_start = initial_start + offset_3;
    Point_3 new_end = initial_end + offset_3;
    return Edge(new_start, new_end, key);
}

AllLines::OutlineAsEdges::OutlineAsEdges(
    const std::vector<std::vector<std::vector<EdgeId>>> &multi_polygons)
    : multi_polygons(multi_polygons) {}

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

const AllLines::EdgeGroup &
AllLines::AllOutlines::get_edge_group(EdgeGroupId edge_group_id) const {
    return edge_groups.at(edge_group_id);
}

const AllLines::OutlineAsEdges &
AllLines::AllOutlines::get_outline(OutlineId outline_id) const {
    return outlines.at(outline_id);
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
    //             next_edge_ids[edge_id] = polygon[(i + 1) % polygon.size()];
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

    // std::cout << "edge_id_to_edge_group_id: " << std::endl;
    // for (EdgeId edge_id(0); edge_id < edges.size(); ++edge_id) {
    //     EdgeGroupId edge_group_id = edge_id_to_edge_group_id[edge_id];
    //     if (edge_id != edge_group_id) {
    //         std::cout << "  Edge " << edge_id << " -> Edge group "
    //                   << edge_id_to_edge_group_id[edge_id] << std::endl;
    //     }
    // }

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

    return AllOutlines(edges, prev_edge_ids, next_edge_ids, edge_groups,
                       edge_id_to_edge_group_id, outlines,
                       edge_id_to_outline_id, optim_units,
                       edge_group_id_to_optim_unit_id);
}

void AllLines::AllOutlines::compute_metrics(
    AllLines::EdgeGroupId edge_group_id, std::vector<double> offsets,
    UnitVector_2 offset_direction, const PtsStructs::StoragePtr las_points,
    std::vector<double> &metrics,
    std::vector<std::map<AllLines::EdgeId, double>> &configs) const {
    // TODO: Compute the metric efficiently for all offsets by projecting
    // only once and then checking whether the projected points are within
    // the edge segment for each offset

    // Get the edge group
    EdgeGroup edge_group = get_edge_group(edge_group_id);
    if (edge_group.edge_ids.empty()) {
        throw std::runtime_error("Edge group has no edges");
    }

    // Separate the positive and negative offsets and sort them by absolute
    // value. Also keep the original indices of the offsets to be able to return
    // the metrics in the correct order
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
    // std::cout << "Computing configurations for all offsets" << std::endl;
    std::vector<std::map<AllLines::EdgeId, double>> pos_configurations;
    std::vector<std::map<AllLines::EdgeId, double>> neg_configurations;
    AllOutlinesPtr all_outlines_ptr = std::make_shared<AllOutlines>(*this);
    LineMoverSimple pos_line_mover(all_outlines_ptr, edge_group_id,
                                   offset_direction, pos_offsets);
    LineMoverSimple neg_line_mover(all_outlines_ptr, edge_group_id,
                                   -offset_direction, neg_offsets);
    pos_line_mover.compute_all();
    neg_line_mover.compute_all();
    pos_line_mover.get_computed_shifts(pos_configurations);
    neg_line_mover.get_computed_shifts(neg_configurations);

    // Gather offsets and configurations together
    // std::cout << "Gathering offsets and configurations together" <<
    // std::endl;
    std::vector<double> pos_neg_offsets = pos_offsets;
    for (double neg_offset : neg_offsets) {
        pos_neg_offsets.push_back(-neg_offset);
    }
    std::vector<std::map<AllLines::EdgeId, double>> pos_neg_configurations =
        pos_configurations;
    for (const auto &neg_config : neg_configurations) {
        std::map<AllLines::EdgeId, double> pos_neg_config;
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
    // std::cout
    //     << "Gathering the edges that are shifted in any of the
    //     configurations"
    //     << std::endl;
    std::set<AllLines::EdgeId> _encountered_edge_ids;
    for (const auto &config : pos_neg_configurations) {
        for (const auto &pair : config) {
            _encountered_edge_ids.insert(pair.first);
        }
    }

    // Make sure all the neighbours of the encountered edges are also included
    std::set<AllLines::EdgeId> criterion_edge_ids(_encountered_edge_ids);
    for (const auto &edge_id : _encountered_edge_ids) {
        criterion_edge_ids.insert(get_prev_edge_id(edge_id));
        criterion_edge_ids.insert(get_next_edge_id(edge_id));
    }

    // std::cout << "Number of edges in the edge group: "
    //           << edge_group.edge_ids.size() << std::endl;
    // std::cout << "Number of edges that are shifted in any configuration: "
    //           << _encountered_edge_ids.size() << std::endl;
    // std::cout << "Number of edges that are shifted or are neighbours of "
    //           << "shifted edges in any configuration: "
    //           << criterion_edge_ids.size() << std::endl;

    // Compute the bounding box of all cases
    // std::cout << "Computing bounding box of all cases" << std::endl;
    Bbox_2 all_cases_bbox;
    for (const auto &config : pos_neg_configurations) {
        for (const auto &edge_id : criterion_edge_ids) {
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
    // std::cout << "Bounding box of all cases: " << all_cases_bbox <<
    // std::endl;

    // Select all the necessary LAS points for the metric computation
    // std::cout << "Selecting all the necessary LAS points for the metric "
    //              "computation"
    //           << std::endl;
    const std::vector<std::size_t> current_las_indices =
        las_points->get_kd_tree_2d()->search_indices_in_box(all_cases_bbox,
                                                            0.0);

    // std::cout << "Number of LAS points in the bounding box: "
    //           << current_las_indices.size() << std::endl;

    std::vector<PtsStructs::PointId> current_las_point_ids;
    current_las_point_ids.reserve(current_las_indices.size());
    for (std::size_t idx : current_las_indices) {
        current_las_point_ids.emplace_back(PtsStructs::PointId(idx));
    }

    // Compute the weights for the LAS points
    // std::cout << "Computing weights for the LAS points" << std::endl;
    std::vector<double> weights;
    std::vector<Vector_2> point_inner_dirs;
    compute_weights(las_points, current_las_point_ids, weights,
                    point_inner_dirs);
    // std::cout << "current_las_point_ids.size(): "
    //           << current_las_point_ids.size() << std::endl;
    // std::cout << "weights.size(): " << weights.size() << std::endl;

    // Prepare the class to compute the criterion
    std::vector<Point_2> selected_las_points(current_las_point_ids.size());
    for (size_t i = 0; i < current_las_point_ids.size(); ++i) {
        const auto &point_id = current_las_point_ids[i];
        selected_las_points[i] = las_points->get_point_2d(point_id);
    }
    LinearCriterion criterion(selected_las_points, weights, point_inner_dirs);

    // Compute the metric for each offset
    // std::cout << "Computing metric for each offset" << std::endl;
    metrics.resize(offsets.size(), 0.0);
    configs.resize(offsets.size());
    for (std::size_t i = 0; i < pos_neg_offsets.size(); ++i) {
        double offset = pos_neg_offsets[i];
        const auto &config = pos_neg_configurations[i];

        // Build the segments for the current configuration
        std::map<AllLines::EdgeId, Edge> current_edges;
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
        std::vector<UnitVector_2> segments_inner_normals;
        segments.reserve(criterion_edge_ids.size());
        segments_initial_length.reserve(criterion_edge_ids.size());
        segments_inner_normals.reserve(criterion_edge_ids.size());

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
            // std::cout << "initial_start: " << std::setprecision(17)
            //           << edge.get_initial_start() << std::endl;
            // std::cout << "initial_end:   " << std::setprecision(17)
            //           << edge.get_initial_end() << std::endl;
            Vector_2 initial_start_to_end_2d(initial_start_to_end.x(),
                                             initial_start_to_end.y());
            segments_initial_length.push_back(
                std::sqrt(initial_start_to_end_2d.squared_length()));

            Vector_2 inner_normal =
                initial_start_to_end_2d.perpendicular(CGAL::COUNTERCLOCKWISE);
            // std::cout << "inner_normal:   " << std::setprecision(17)
            //           << inner_normal << std::endl;
            segments_inner_normals.push_back(inner_normal);
        }

        // Compute the metric for the current configuration
        double metric = criterion.evaluate_segments(
            segments, segments_initial_length, segments_inner_normals);
        metrics.at(pos_neg_offset_indices[i]) = metric;
        configs.at(pos_neg_offset_indices[i]) = config;

        // std::cout << "Offset: " << offset << ", Metric: " << metric
        //           << std::endl;
    }
    // std::cout << "Done computing metric for each offset" << std::endl;
}

void AllLines::AllOutlines::compute_optimal_offset(
    EdgeGroupId edge_group_id, double max_absolute_offset,
    uint initial_samples_one_side, UnitVector_2 offset_direction,
    PtsStructs::StoragePtr las_points, double &best_offset,
    std::map<AllLines::EdgeId, double> &best_config) const {

    // std::cout << "Computing optimal offset for edge group " << edge_group_id
    //           << std::endl;

    /* ---------------------------------------------------------------------- */
    /*                              Initial step                              */
    /* ---------------------------------------------------------------------- */

    double min_offset = -max_absolute_offset;
    double max_offset = max_absolute_offset;

    double current_precision =
        (max_offset - min_offset) / (2 * initial_samples_one_side);

    // Build the list of offsets to evaluate
    std::vector<double> offsets;
    for (uint sample = 0; sample <= 2 * initial_samples_one_side; ++sample) {
        double offset = min_offset + sample * current_precision;
        offsets.push_back(offset);
    }

    // Compute the metrics for all offsets
    std::vector<double> metrics;
    std::vector<std::map<AllLines::EdgeId, double>> configs;
    compute_metrics(edge_group_id, offsets, offset_direction, las_points,
                    metrics, configs);

    // Find the best offset
    size_t best_offset_index = -1;
    double best_metric = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < offsets.size(); ++i) {
        // Add a small penalty to the metric based on the absolute value of the
        // offset to prefer smaller offsets in case of ties
        double metric = metrics[i] + std::abs(offsets[i]) * 1e-6;
        if (metric < best_metric) {
            best_metric = metric;
            best_offset_index = i;
        }
    }

    /* ---------------------------------------------------------------------- */
    /*                            Refinement steps                            */
    /* ---------------------------------------------------------------------- */

    while (current_precision > EDGE_CRITERION_FINAL_PRECISION) {
        // std::cout << "Refining optimal offset with precision "
        //           << current_precision << " and best offset index "
        //           << best_offset_index << std::endl;

        double new_precision =
            current_precision / (EDGE_CRITERION_REFINEMENT_SAMPLES + 1);

        std::vector<double> new_offsets;
        std::vector<double> new_metrics;
        std::vector<std::map<AllLines::EdgeId, double>> new_configs;

        // Build the lists of offsets to evaluate around the best offset
        std::vector<double> prev_offsets;
        if (best_offset_index > 0) {
            new_offsets.push_back(offsets[best_offset_index - 1]);
            new_metrics.push_back(metrics[best_offset_index - 1]);
            new_configs.push_back(configs[best_offset_index - 1]);

            double prev_offset = offsets[best_offset_index - 1];
            for (uint sample = 1; sample <= EDGE_CRITERION_REFINEMENT_SAMPLES;
                 ++sample) {
                double offset = prev_offset + sample * new_precision;
                prev_offsets.push_back(offset);
            }

            std::vector<double> prev_metrics;
            std::vector<std::map<AllLines::EdgeId, double>> prev_configs;

            compute_metrics(edge_group_id, prev_offsets, offset_direction,
                            las_points, prev_metrics, prev_configs);

            new_offsets.insert(new_offsets.end(), prev_offsets.begin(),
                               prev_offsets.end());
            new_metrics.insert(new_metrics.end(), prev_metrics.begin(),
                               prev_metrics.end());
            new_configs.insert(new_configs.end(), prev_configs.begin(),
                               prev_configs.end());
        }

        new_offsets.push_back(offsets[best_offset_index]);
        new_metrics.push_back(metrics[best_offset_index]);
        new_configs.push_back(configs[best_offset_index]);

        std::vector<double> next_offsets;
        if (best_offset_index < offsets.size() - 1) {
            double next_offset = offsets[best_offset_index + 1];
            for (uint sample = EDGE_CRITERION_REFINEMENT_SAMPLES; sample >= 1;
                 --sample) {
                double offset = next_offset - sample * new_precision;
                next_offsets.push_back(offset);
            }

            std::vector<double> next_metrics;
            std::vector<std::map<AllLines::EdgeId, double>> next_configs;

            compute_metrics(edge_group_id, next_offsets, offset_direction,
                            las_points, next_metrics, next_configs);

            new_offsets.insert(new_offsets.end(), next_offsets.begin(),
                               next_offsets.end());
            new_metrics.insert(new_metrics.end(), next_metrics.begin(),
                               next_metrics.end());
            new_configs.insert(new_configs.end(), next_configs.begin(),
                               next_configs.end());

            new_offsets.push_back(offsets[best_offset_index + 1]);
            new_metrics.push_back(metrics[best_offset_index + 1]);
            new_configs.push_back(configs[best_offset_index + 1]);
        }

        // Select the best offset among the new offsets and the current best
        // offset
        size_t new_best_offset_index = -1;
        double new_best_metric = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < new_metrics.size(); ++i) {
            double metric = new_metrics[i] + std::abs(new_offsets[i]) * 1e-6;
            if (metric < new_best_metric) {
                new_best_metric = metric;
                new_best_offset_index = i;
            }
        }

        // Update everything
        offsets = new_offsets;
        metrics = new_metrics;
        configs = new_configs;
        best_offset_index = new_best_offset_index;
        best_metric = new_best_metric;
        current_precision = new_precision;
    }

    best_offset = offsets[best_offset_index];
    best_config = configs[best_offset_index];

    // std::cout << "Done computing optimal offset for edge group "
    //           << edge_group_id << std::endl;
}

void AllLines::AllOutlines::optimize_unit(
    const PtsStructs::StoragePtr las_points,
    const OptimizationUnitId &optim_unit_id) {
    const auto &optim_unit = optim_units[optim_unit_id];
    // std::cout << "Optimizing unit " << optim_unit_id << " with "
    //           << optim_unit.edge_group_ids.size() << " edge groups"
    //           << std::endl;

    // Order the edge groups in the optimization unit based on the length of
    // their edges, starting with the longest
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

    for (const auto &[edge_group_id, length] : edge_groups_with_length) {
        // Compute the offset direction based on the edge group
        EdgeGroup edge_group = get_edge_group(edge_group_id);
        if (edge_group.edge_ids.empty()) {
            throw std::runtime_error("Edge group has no edges");
        }
        EdgeId edge_id = edge_group.edge_ids[0];
        Edge edge = get_edge(edge_id);
        UnitVector_2 offset_direction = edge.get_normal();

        // std::cout << "Optimizing edge group " << edge_group_id << " with "
        //           << edge_group.edge_ids.size()
        //           << " edges and offset direction " << offset_direction
        //           << std::endl;
        // std::cout << "  Keys of edges in the group: ";
        // for (const auto &edge_id : edge_group.edge_ids) {
        //     std::cout << get_edge(edge_id).get_key() << " ";
        // }
        // std::cout << std::endl;

        double best_offset;
        std::map<AllLines::EdgeId, double> best_config;
        compute_optimal_offset(edge_group_id, EDGE_CRITERION_OFFSET_MAX,
                               EDGE_CRITERION_INITIAL_SAMPLES_ONE_SIDE,
                               offset_direction, las_points, best_offset,
                               best_config);

        // Apply the best configuration to the edges in the optimization unit
        for (const auto &[edge_id, offset] : best_config) {
            edges[edge_id] =
                edges[edge_id].translated(offset * offset_direction);
        }
    }
}

void AllLines::AllOutlines::optimize_all_units(
    const PtsStructs::StoragePtr las_points) {
    ProgressBarTotal progress_bar(optim_units.size(), "Optimizing edge groups");
    for (OptimizationUnitId optim_unit_id(0);
         optim_unit_id < optim_units.size(); ++optim_unit_id) {
        optimize_unit(las_points, optim_unit_id);
        progress_bar.increment(1);
    }
    progress_bar.finish();
}

void AllLines::AllOutlines::get_multipolygons(
    std::vector<OutlineAsEdges> &outline_as_edges,
    std::vector<MultiPolygonZ> &multi_polygons) const {

    multi_polygons.clear();
    multi_polygons.reserve(outline_as_edges.size());
    for (size_t i = 0; i < outline_as_edges.size(); ++i) {
        OutlineAsEdges outline = outline_as_edges[i];
        std::vector<PolygonZ> polygons;

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
        multi_polygons.emplace_back(MultiPolygonZ(polygons));
    }
}

void AllLines::compute_roofprints(
    const std::string &input_las_file,
    const std::string &input_bd_topo_edges_file,
    const std::string &input_bd_topo_intersections_file,
    const std::string &output_roofprints_file, uint iterations,
    bool overwrite) {

    if (std::filesystem::exists(output_roofprints_file) && !overwrite) {
        throw std::runtime_error("Output file already exists: " +
                                 output_roofprints_file);
    }

    std::filesystem::create_directories(
        std::filesystem::path(output_roofprints_file).parent_path());

    // Read the LAS file and get the point view
    std::cout << "Reading LAS file..." << std::endl;
    NewLasReader las_reader(input_las_file);
    std::cout << "Successfully read LAS file with "
              << las_reader.points->point_count() << " points." << std::endl;

    // Build the 2D KD-tree for the LAS points to enable efficient spatial
    // queries
    std::cout << "Building 2D KD-tree for LAS points..." << std::endl;
    las_reader.points->build_kd_tree_2d();

    // Read the building outlines from the BD TOPO file
    std::cout << "Reading building outlines from BD TOPO files..." << std::endl;
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

    // std::cout << "Intersections between edges: " << std::endl;
    // for (const auto &intersection : _intersections) {
    //     std::cout << "  Edge " << intersection.first << " intersects with
    //     edge "
    //               << intersection.second << std::endl;
    // }

    // Format the edges for the AllOutlines data structures
    EdgeVector<Edge> edges;
    std::map<uint32_t, EdgeId> edge_key_map;
    for (const auto &edge : initial_edges) {
        edges.push_back(Edge(edge.start, edge.end, edge.edge_key));
        edge_key_map[edge.edge_key] = EdgeId(edges.size() - 1);
        // std::cout << "Mapped edge key " << edge.edge_key << " to edge id "
        //           << edge_key_map[edge.edge_key] << std::endl;
    }
    std::vector<std::pair<EdgeId, EdgeId>> intersections;
    for (const auto &intersection : _intersections) {
        EdgeId edge_id_1 = EdgeId(intersection.first);
        EdgeId edge_id_2 = EdgeId(intersection.second);
        intersections.push_back({edge_id_1, edge_id_2});
        // uint32_t edge_key_1 = intersection.first;
        // uint32_t edge_key_2 = intersection.second;
        // std::cout << "Mapping intersection between edge keys " << edge_key_1
        //           << " and " << edge_key_2 << std::endl;
        // intersections.push_back(
        //     {edge_key_map.at(edge_key_1), edge_key_map.at(edge_key_2)});
    }

    // Rebuild the MultiPolygon hierarchy based on the edges and their
    // building, polygon, and ring indices
    std::map<std::string, std::vector<std::vector<std::vector<EdgeId>>>>
        building_id_to_multi_polygons;
    std::map<std::string, std::vector<std::vector<std::vector<bool>>>>
        found_edges;
    for (const auto &edge : initial_edges) {
        auto building_id = edge.building_id;
        auto polygon_idx = edge.polygon_idx;
        auto ring_idx = edge.ring_idx;
        auto edge_idx = edge.edge_idx;
        auto edge_key = edge.edge_key;

        if (building_id_to_multi_polygons[building_id].size() <= polygon_idx) {
            building_id_to_multi_polygons[building_id].resize(polygon_idx + 1);
            found_edges[building_id].resize(polygon_idx + 1);
        }
        if (building_id_to_multi_polygons[building_id][polygon_idx].size() <=
            ring_idx) {
            building_id_to_multi_polygons[building_id][polygon_idx].resize(
                ring_idx + 1);
            found_edges[building_id][polygon_idx].resize(ring_idx + 1);
        }
        if (building_id_to_multi_polygons[building_id][polygon_idx][ring_idx]
                .size() <= edge_idx) {
            building_id_to_multi_polygons[building_id][polygon_idx][ring_idx]
                .resize(edge_idx + 1);
            found_edges[building_id][polygon_idx][ring_idx].resize(edge_idx + 1,
                                                                   false);
        }
        building_id_to_multi_polygons[building_id][polygon_idx][ring_idx]
                                     [edge_idx] = edge_key_map.at(edge_key);
        found_edges[building_id][polygon_idx][ring_idx][edge_idx] = true;
    }

    // Check for any missing edges in the hierarchy and throw an error if
    // any are found
    for (const auto &[building_id, multi_polygons] :
         building_id_to_multi_polygons) {
        for (std::size_t polygon_idx = 0; polygon_idx < multi_polygons.size();
             ++polygon_idx) {
            for (std::size_t ring_idx = 0;
                 ring_idx < multi_polygons[polygon_idx].size(); ++ring_idx) {
                for (std::size_t edge_idx = 0;
                     edge_idx < multi_polygons[polygon_idx][ring_idx].size();
                     ++edge_idx) {
                    if (!found_edges[building_id][polygon_idx][ring_idx]
                                    [edge_idx]) {
                        throw std::runtime_error(
                            "Warning: Missing edge for building " +
                            building_id + ", polygon " +
                            std::to_string(polygon_idx) + ", ring " +
                            std::to_string(ring_idx));
                    }
                }
            }
        }
    }

    // Build the expected structure
    OutlineVector<OutlineAsEdges> outlines;
    std::vector<OutlineAsEdges> outlines_as_vec;
    std::vector<std::string> outline_building_ids;
    for (const auto &[building_id, multi_polygons] :
         building_id_to_multi_polygons) {
        std::vector<std::vector<std::vector<EdgeId>>> outline_multi_polygons;
        for (const auto &multi_polygon : multi_polygons) {
            std::vector<std::vector<EdgeId>> outline_polygons;
            for (const auto &polygon : multi_polygon) {
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
                outline_multi_polygons.push_back(outline_polygons);
            } else {
                std::cerr
                    << "Warning: MultiPolygon with no polygons in building "
                    << building_id
                    << ", skipping this multi-polygon in the outline."
                    << std::endl;
            }
        }
        if (!outline_multi_polygons.empty()) {
            OutlineAsEdges outline_as_edges(outline_multi_polygons);
            outlines.push_back(outline_as_edges);
            outlines_as_vec.push_back(outline_as_edges);
            outline_building_ids.push_back(building_id);
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
    for (uint i = 0; i < iterations; ++i) {
        std::cout << "Optimization iteration " << (i + 1) << " / " << iterations
                  << std::endl;
        all_outlines.optimize_all_units(las_reader.points);
    }

    // Write the roofprints to a Parquet file
    std::cout << "Writing roofprints to Parquet file..." << std::endl;
    std::vector<MultiPolygonZ> optimized_outlines;
    all_outlines.get_multipolygons(outlines_as_vec, optimized_outlines);
    std::vector<MultiPolygonZWithAttributes> optimized_outlines_with_attributes;
    for (size_t i = 0; i < optimized_outlines.size(); ++i) {
        optimized_outlines_with_attributes.push_back(
            {optimized_outlines[i], outline_building_ids[i],
             OutlineSource::Id::Unknown});
    }

    auto write_status = write_geoms_to_parquet(
        optimized_outlines_with_attributes, output_roofprints_file, overwrite);

    if (!write_status.ok()) {
        std::cerr << "Error writing roofprints to Parquet: "
                  << write_status.ToString() << std::endl;
        throw std::runtime_error("Failed to write roofprints to Parquet");
    }
}