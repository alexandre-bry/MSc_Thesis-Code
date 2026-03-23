#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "utils/cgal.hpp"
#include "utils/strong_types.hpp"

namespace NewRoofprints {
struct EdgeIdTag {};
using EdgeId = StrongType<EdgeIdTag, std::size_t>;
template <typename T> using EdgeVector = StrongTypedVector<EdgeId, T>;

class AllOutlines;
using AllOutlinesPtr = std::shared_ptr<AllOutlines>;

class Edge {
  private:
    Point_2 initial_start;
    Point_2 initial_end;
    Line_2 line;
    Vector_2 direction;

  public:
    Edge(Point_2 initial_start, Point_2 initial_end);

    Point_2 get_initial_start() const;
    Point_2 get_initial_end() const;
    Vector_2 get_direction() const;
    Line_2 to_line() const;

    void translate(Vector_2 offset);
};

struct EdgeGroupIdTag {};
using EdgeGroupId = StrongType<EdgeGroupIdTag, std::size_t>;
template <typename T> using EdgeGroupVector = StrongTypedVector<EdgeGroupId, T>;

struct EdgeGroup {
    std::vector<EdgeId> edge_ids;

    EdgeGroup() = default;
    explicit EdgeGroup(const std::vector<EdgeId> &edge_ids);
};

struct OutlineIdTag {};
using OutlineId = StrongType<OutlineIdTag, std::size_t>;
template <typename T> using OutlineVector = StrongTypedVector<OutlineId, T>;

struct OutlineAsEdges {
    std::vector<std::vector<EdgeId>> polygons;

    OutlineAsEdges() = default;
    explicit OutlineAsEdges(const std::vector<std::vector<EdgeId>> &polygons);
};

struct SuperOutlineIdTag {};
using SuperOutlineId = StrongType<SuperOutlineIdTag, std::size_t>;
template <typename T>
using SuperOutlineVector = StrongTypedVector<SuperOutlineId, T>;

struct SuperOutline {
    std::vector<OutlineId> outline_ids;

    SuperOutline() = default;
    explicit SuperOutline(const std::vector<OutlineId> &outline_ids);
};

class AllOutlines {
    // Edges
    EdgeVector<Edge> edges;
    EdgeVector<EdgeId> prev_edge_ids;
    EdgeVector<EdgeId> next_edge_ids;

    // Edge groups
    EdgeGroupVector<EdgeGroup> edge_groups;
    EdgeVector<EdgeGroupId> edge_id_to_edge_group_id;
    EdgeGroupVector<EdgeGroupId> prev_edge_group_ids;
    EdgeGroupVector<EdgeGroupId> next_edge_group_ids;

    // Outlines
    OutlineVector<OutlineAsEdges> outlines;
    EdgeGroupVector<OutlineId> edge_group_id_to_outline_id;

    // Super outlines (groups of outlines that must be optimized together)
    SuperOutlineVector<SuperOutline> super_outlines;
    OutlineVector<SuperOutlineId> outline_id_to_super_outline_id;

  public:
    AllOutlines() = default;
    AllOutlines(
        const EdgeVector<Edge> &edges, const EdgeVector<EdgeId> &prev_edge_ids,
        const EdgeVector<EdgeId> &next_edge_ids,
        const EdgeGroupVector<EdgeGroup> &edge_groups,
        const EdgeVector<EdgeGroupId> &edge_id_to_edge_group_id,
        const EdgeGroupVector<EdgeGroupId> &prev_edge_group_ids,
        const EdgeGroupVector<EdgeGroupId> &next_edge_group_ids,
        const OutlineVector<OutlineAsEdges> &outlines,
        const EdgeGroupVector<OutlineId> &edge_group_id_to_outline_id,
        const SuperOutlineVector<SuperOutline> &super_outlines,
        const OutlineVector<SuperOutlineId> &outline_id_to_super_outline_id);

    const Edge &get_edge(EdgeId edge_id) const;
    Edge &get_edge(EdgeId edge_id);

    EdgeGroupId get_edge_group_id(EdgeId edge_id) const;
    OutlineId get_outline_id(EdgeGroupId edge_group_id) const;
    SuperOutlineId get_super_outline_id(OutlineId outline_id) const;

    EdgeId get_prev_edge_id(EdgeId edge_id) const;
    EdgeId get_next_edge_id(EdgeId edge_id) const;

    EdgeGroupId get_prev_edge_group_id(EdgeGroupId edge_group_id) const;
    EdgeGroupId get_next_edge_group_id(EdgeGroupId edge_group_id) const;

    Point_2 get_edge_start(EdgeId edge_id) const;
    Point_2 get_edge_end(EdgeId edge_id) const;

    EdgeId edge_count() const;
    EdgeGroupId edge_group_count() const;
    OutlineId outline_count() const;
    SuperOutlineId super_outline_count() const;
};

AllOutlines make_all_outlines(const EdgeVector<Edge> &edges,
                              const OutlineVector<OutlineAsEdges> &outlines);

void compute_roofprints(const std::string &input_las_file,
                        const std::string &input_bd_topo_edges_file,
                        const std::string &input_bd_topo_intersections_file,
                        const std::string &output_roofprints_file,
                        double las_buffer_distance,
                        double outline_buffer_distance, bool overwrite);

} // namespace NewRoofprints