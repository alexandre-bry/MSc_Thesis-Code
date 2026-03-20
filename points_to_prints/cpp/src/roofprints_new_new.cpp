#include "roofprints_new_new.hpp"

#include <vector>

NewRoofprints::Edge::Edge(Point_2 initial_start, Point_2 initial_end)
    : initial_start(initial_start), initial_end(initial_end),
      line(initial_start, initial_end) {
    direction = line.direction().to_vector();
    direction /= std::sqrt(direction.squared_length());
}

Point_2 NewRoofprints::Edge::get_initial_start() const { return initial_start; }
Point_2 NewRoofprints::Edge::get_initial_end() const { return initial_end; }
Vector_2 NewRoofprints::Edge::get_direction() const { return direction; }
Line_2 NewRoofprints::Edge::to_line() const { return line; }

void NewRoofprints::Edge::translate(Vector_2 offset) {
    line = Line_2(line.point(0) + offset, line.direction());
}

NewRoofprints::EdgeGroup::EdgeGroup(const std::vector<EdgeId> &edge_ids)
    : edge_ids(edge_ids) {}

NewRoofprints::OutlineAsEdges::OutlineAsEdges(
    const std::vector<std::vector<EdgeId>> &polygons)
    : polygons(polygons) {}

NewRoofprints::SuperOutline::SuperOutline(
    const std::vector<OutlineId> &outline_ids)
    : outline_ids(outline_ids) {}

NewRoofprints::AllOutlines::AllOutlines(
    const EdgeVector<Edge> &edges, const EdgeVector<EdgeId> &prev_edge_ids,
    const EdgeVector<EdgeId> &next_edge_ids,
    const EdgeGroupVector<EdgeGroup> &edge_groups,
    const EdgeVector<EdgeGroupId> &edge_id_to_edge_group_id,
    const EdgeGroupVector<EdgeGroupId> &prev_edge_group_ids,
    const EdgeGroupVector<EdgeGroupId> &next_edge_group_ids,
    const OutlineVector<OutlineAsEdges> &outlines,
    const EdgeGroupVector<OutlineId> &edge_group_id_to_outline_id,
    const SuperOutlineVector<SuperOutline> &super_outlines,
    const OutlineVector<SuperOutlineId> &outline_id_to_super_outline_id)
    : edges(edges), prev_edge_ids(prev_edge_ids), next_edge_ids(next_edge_ids),
      edge_groups(edge_groups),
      edge_id_to_edge_group_id(edge_id_to_edge_group_id),
      prev_edge_group_ids(prev_edge_group_ids),
      next_edge_group_ids(next_edge_group_ids), outlines(outlines),
      edge_group_id_to_outline_id(edge_group_id_to_outline_id),
      super_outlines(super_outlines),
      outline_id_to_super_outline_id(outline_id_to_super_outline_id) {}

const NewRoofprints::Edge &
NewRoofprints::AllOutlines::get_edge(EdgeId edge_id) const {
    return edges.at(edge_id);
}

NewRoofprints::Edge &NewRoofprints::AllOutlines::get_edge(EdgeId edge_id) {
    return edges.at(edge_id);
}

NewRoofprints::EdgeGroupId
NewRoofprints::AllOutlines::get_edge_group_id(EdgeId edge_id) const {
    return edge_id_to_edge_group_id.at(edge_id);
}

NewRoofprints::OutlineId
NewRoofprints::AllOutlines::get_outline_id(EdgeGroupId edge_group_id) const {
    return edge_group_id_to_outline_id.at(edge_group_id);
}

NewRoofprints::SuperOutlineId
NewRoofprints::AllOutlines::get_super_outline_id(OutlineId outline_id) const {
    return outline_id_to_super_outline_id.at(outline_id);
}

NewRoofprints::EdgeId
NewRoofprints::AllOutlines::get_prev_edge_id(EdgeId edge_id) const {
    return prev_edge_ids.at(edge_id);
}

NewRoofprints::EdgeId
NewRoofprints::AllOutlines::get_next_edge_id(EdgeId edge_id) const {
    return next_edge_ids.at(edge_id);
}

NewRoofprints::EdgeGroupId NewRoofprints::AllOutlines::get_prev_edge_group_id(
    EdgeGroupId edge_group_id) const {
    return prev_edge_group_ids.at(edge_group_id);
}

NewRoofprints::EdgeGroupId NewRoofprints::AllOutlines::get_next_edge_group_id(
    EdgeGroupId edge_group_id) const {
    return next_edge_group_ids.at(edge_group_id);
}

Point_2 NewRoofprints::AllOutlines::get_edge_start(EdgeId edge_id) const {
    return get_edge(edge_id).get_initial_start();
}

Point_2 NewRoofprints::AllOutlines::get_edge_end(EdgeId edge_id) const {
    return get_edge(edge_id).get_initial_end();
}

NewRoofprints::EdgeId NewRoofprints::AllOutlines::edge_count() const {
    return edges.size_as_strong_index();
}

NewRoofprints::EdgeGroupId
NewRoofprints::AllOutlines::edge_group_count() const {
    return edge_groups.size_as_strong_index();
}

NewRoofprints::OutlineId NewRoofprints::AllOutlines::outline_count() const {
    return outlines.size_as_strong_index();
}

NewRoofprints::SuperOutlineId
NewRoofprints::AllOutlines::super_outline_count() const {
    return super_outlines.size_as_strong_index();
}

NewRoofprints::AllOutlines NewRoofprints::make_all_outlines(
    const EdgeVector<Edge> &edges,
    const OutlineVector<OutlineAsEdges> &outlines) {

    // Find the previous and next edge for each edge
    std::vector<EdgeId> prev_edge_ids(edges.size());
    std::vector<EdgeId> next_edge_ids(edges.size());
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

    // TODO
}