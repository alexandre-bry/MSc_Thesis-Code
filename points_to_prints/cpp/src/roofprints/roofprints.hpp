#pragma once

#include <string>

#include "../edge_matching/topology.hpp"

namespace EdgeMatching {

class AllRoofprints : public AllOutlines {
  protected:
    void
    compute_weights(PtsStructs::StoragePtr las_points,
                    const std::vector<PtsStructs::PointId> &point_ids,
                    std::vector<double> &weights,
                    std::vector<Vector_2> &point_inward_dirs) const override;

    std::unique_ptr<ICriterion> create_criterion(
        std::vector<Point_2> points, std::vector<double> weights,
        std::vector<Vector_2> point_inward_dirs,
        std::vector<LASclassification::Value> point_classes) const override;

    void
    find_relevant_points(const std::vector<std::map<EdgeId, double>> &configs,
                         const std::set<EdgeMatching::EdgeId> edge_ids,
                         const UnitVector_2 offset_direction,
                         const PtsStructs::StoragePtr las_points,
                         std::vector<std::size_t> &las_indices) const override;

  public:
    AllRoofprints(const EdgeVector<Edge> &edges,
                  const OutlineVector<OutlineAsEdges> &outlines,
                  const std::vector<std::pair<EdgeId, EdgeId>> &intersections)
        : AllOutlines(edges, outlines, intersections) {}

    void prepare_offsets(std::vector<double> &offsets) const override;
};

struct ComputeRoofprintsOptions {
    std::string input_las_file;
    std::string input_bd_topo_edges_file;
    std::string input_bd_topo_intersections_file;
    std::string output_roofprints_template_file;
    uint iterations;
    bool overwrite;
};

arrow::Status
compute_roofprints(const std::string &input_las_file,
                   const std::string &input_bd_topo_edges_file,
                   const std::string &input_bd_topo_intersections_file,
                   const std::string &output_roofprints_template_file,
                   uint iterations, bool overwrite);

} // namespace EdgeMatching
