#pragma once

#include <cstddef>
#include <cstdint>
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
    uint32_t key;
    Point_3 initial_start;
    Point_3 initial_end;
    Line_2 line;
    UnitVector_2 direction;

  public:
    Edge(Point_3 initial_start, Point_3 initial_end, uint32_t key);

    uint32_t get_key() const { return key; }
    Point_3 get_initial_start() const;
    Point_3 get_initial_end() const;
    UnitVector_2 get_direction() const;
    UnitVector_2 get_normal() const;
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

/**
 * @brief Structure representing an outline as a collection of multi polygons,
 * where: each multi polygon is represented as a sequence of polygons, every
 * polygon is represented as a sequence of rings, and every ring is represented
 * as a sequence of edge IDs.
 *
 */
struct OutlineAsEdges {
    std::vector<std::vector<std::vector<EdgeId>>> multi_polygons;

    OutlineAsEdges() = default;
    explicit OutlineAsEdges(
        const std::vector<std::vector<std::vector<EdgeId>>> &multi_polygons);
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
  private:
    // The organization of the outlines for computations is the following:
    // - Edges are the basic elements, they all have a previous and next
    // neighbour because they are part of polygons with at least 3 edges.
    // - Edge groups are groups of edges that are grouped together because their
    // intersections were segments.
    // - Outlines are a simple representation of multi polygons that is only
    // used after processing to reconstruct the final multi polygons.
    // - Optimization units are sets of edge groups that must be optimized
    // together because they contain edges in the same buildings.

    // Edges
    EdgeVector<Edge> edges;
    EdgeVector<EdgeId> prev_edge_ids;
    EdgeVector<EdgeId> next_edge_ids;

    // Edge groups (groups of edges that must be optimized together)
    EdgeGroupVector<EdgeGroup> edge_groups;
    EdgeVector<EdgeGroupId> edge_id_to_edge_group_id;

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
    const EdgeGroup &get_edge_group(EdgeGroupId edge_group_id) const;
    const OutlineAsEdges &get_outline(OutlineId outline_id) const;

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

    void compute_metrics(EdgeGroupId edge_group_id, std::vector<double> offsets,
                         UnitVector_2 offset_direction,
                         const PtsStructs::StoragePtr las_points,
                         std::vector<double> &metrics,
                         std::vector<std::map<EdgeId, double>> &configs) const;

    /**
     * @brief Computes the optimal offset for an edge group by evaluating the
     * given offsets and selecting the one with the best metric. The best metric
     * and the corresponding configuration of shifts for the edges are returned.
     *
     * @param edge_group_id ID of the edge group for which to compute the
     * optimal offset
     * @param max_absolute_offset Maximum absolute value of the offset to
     * evaluate (the function will evaluate offsets in the range
     * [-max_absolute_offset, max_absolute_offset])
     * @param offset_step Step size for evaluating offsets (the function will
     * evaluate offsets at intervals of offset_step within the range defined by
     * max_absolute_offset)
     * @param offset_direction Direction in which to apply the offsets
     * @param las_points Pointer to the LAS points storage
     * @param best_offset Reference to the best offset found
     * @param best_config Reference to the configuration of shifts for the edges
     * corresponding to the best offset found
     */
    void compute_optimal_offset(EdgeGroupId edge_group_id,
                                double max_absolute_offset, double offset_step,
                                UnitVector_2 offset_direction,
                                PtsStructs::StoragePtr las_points,
                                double &best_offset,
                                std::map<EdgeId, double> &best_config) const;

    void optimize_unit(const PtsStructs::StoragePtr las_points,
                       const OptimizationUnitId &optim_unit_id);
    void optimize_all_units(const PtsStructs::StoragePtr las_points);

    void get_multipolygons(std::vector<OutlineAsEdges> &outline_as_edges,
                           std::vector<MultiPolygonZ> &multi_polygons) const;
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