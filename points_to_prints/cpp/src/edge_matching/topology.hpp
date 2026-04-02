#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "../points.hpp"
#include "../utils/cgal.hpp"
#include "../utils/strong_types.hpp"

namespace AllLines {
struct EdgeIdTag {};
using EdgeId = StrongType<EdgeIdTag, std::size_t>;
template <typename T> using EdgeVector = StrongTypedVector<EdgeId, T>;

class AllOutlines;
using AllOutlinesPtr = std::shared_ptr<AllOutlines>;

class Edge {
  private:
    Point_3 initial_start;
    Point_3 initial_end;
    Line_2 line;
    Vector_2 direction;

  public:
    Edge(Point_3 initial_start, Point_3 initial_end);

    Point_3 get_initial_start() const;
    Point_3 get_initial_end() const;
    Vector_2 get_direction() const;
    Line_2 to_line() const;

    void translate(Vector_2 offset);
    Edge translated(Vector_2 offset) const;
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

struct OptimizationUnitIdTag {};
using OptimizationUnitId = StrongType<OptimizationUnitIdTag, std::size_t>;
template <typename T>
using OptimizationUnitVector = StrongTypedVector<OptimizationUnitId, T>;

struct OptimizationUnit {
    std::vector<EdgeGroupId> edge_group_ids;

    OptimizationUnit() = default;
    explicit OptimizationUnit(const std::vector<EdgeGroupId> &edge_group_ids);
};

class AllOutlines {
    // The organization of the outlines for computations is the following:
    // - Edges are the basic elements, they all have a previous and next
    // neighbour because they are part of polygons with at least 3 edges.
    // - Edge groups are groups of edges that are grouped together because their
    // intersections were segments.
    // - Outlines are a simple representation of multipolygons that is only used
    // after processing to reconstruct the final multi polygons.
    // - Optimization units are sets of edge groups that must be optimized
    // together because they contain edges in the same buildings.

    // Edges
    EdgeVector<Edge> edges;           // OK
    EdgeVector<EdgeId> prev_edge_ids; // OK
    EdgeVector<EdgeId> next_edge_ids; // OK

    // Edge groups (groups of edges that must be optimized together)
    EdgeGroupVector<EdgeGroup> edge_groups;           // OK
    EdgeVector<EdgeGroupId> edge_id_to_edge_group_id; // OK

    // Outlines
    OutlineVector<OutlineAsEdges> outlines;
    EdgeVector<OutlineId> edge_id_to_outline_id;

    // Optimization units (set of edge groups that must be optimized together)
    OptimizationUnitVector<OptimizationUnit> optim_units;
    EdgeGroupVector<OptimizationUnitId> edge_group_id_to_optim_unit_id;

  public:
    AllOutlines() = default;
    AllOutlines(const EdgeVector<Edge> &edges,
                const EdgeVector<EdgeId> &prev_edge_ids,
                const EdgeVector<EdgeId> &next_edge_ids,
                const EdgeGroupVector<EdgeGroup> &edge_groups,
                const EdgeVector<EdgeGroupId> &edge_id_to_edge_group_id,
                const OutlineVector<OutlineAsEdges> &outlines,
                const EdgeVector<OutlineId> &edge_id_to_outline_id,
                const OptimizationUnitVector<OptimizationUnit> &optim_units,
                const EdgeGroupVector<OptimizationUnitId>
                    &edge_group_id_to_optim_unit_id);

    const Edge &get_edge(EdgeId edge_id) const;
    Edge &get_edge(EdgeId edge_id);

    OutlineId get_outline_id(EdgeId edge_id) const;
    EdgeGroupId get_edge_group_id(EdgeId edge_id) const;
    OptimizationUnitId get_optim_unit_id(EdgeGroupId edge_group_id) const;

    EdgeId get_prev_edge_id(EdgeId edge_id) const;
    EdgeId get_next_edge_id(EdgeId edge_id) const;

    Point_2 get_edge_start(EdgeId edge_id) const;
    Point_2 get_edge_end(EdgeId edge_id) const;

    EdgeId edge_count() const;
    EdgeGroupId edge_group_count() const;
    OutlineId outline_count() const;
    OptimizationUnitId optim_unit_count() const;

    void optimize_unit(const PtsStructs::StoragePtr las_points,
                       const OptimizationUnitId &optim_unit_id);
    void optimize_all_units(const PtsStructs::StoragePtr las_points);
};

AllOutlines
make_all_outlines(const EdgeVector<Edge> &edges,
                  const OutlineVector<OutlineAsEdges> &outlines,
                  const std::vector<std::pair<EdgeId, EdgeId>> &intersections);

struct ComputeRoofprintsOptions {
    std::string input_las_file;
    std::string input_bd_topo_edges_file;
    std::string input_bd_topo_intersections_file;
    std::string output_roofprints_file;
    bool overwrite;
};

void compute_roofprints(const std::string &input_las_file,
                        const std::string &input_bd_topo_edges_file,
                        const std::string &input_bd_topo_intersections_file,
                        const std::string &output_roofprints_file,
                        bool overwrite);

} // namespace AllLines