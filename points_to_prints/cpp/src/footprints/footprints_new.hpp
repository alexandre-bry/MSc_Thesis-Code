#pragma once

#include <string>

#include "../edge_matching/topology.hpp"
#include "points_selection.hpp"

namespace EdgeMatching {

class AllFootprints : public AllOutlines {
    OutlineVector<std::reference_wrapper<const PointSelection::RoofBuilding>>
        outline_id_to_building;

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
    AllFootprints(
        const EdgeVector<Edge> &edges,
        const OutlineVector<OutlineAsEdges> &outlines,
        const std::vector<std::pair<EdgeId, EdgeId>> &intersections,
        const OutlineVector<
            std::reference_wrapper<const PointSelection::RoofBuilding>>
            &outline_id_to_building)
        : AllOutlines(edges, outlines, intersections),
          outline_id_to_building(outline_id_to_building) {}

    void prepare_offsets(std::vector<double> &offsets) const override;
};

struct ComputeFootprintsOptions {
    std::string input_roofprints_file;
    std::string input_lod22_file;
    std::string input_points_file;
    std::string output_footprints_template_file;
    uint iterations;
    bool overwrite;
};

arrow::Status
compute_footprints(const std::string &input_roofprints_file,
                   const std::string &input_lod22_file,
                   const std::string &input_points_file,
                   const std::string &output_footprints_template_file,
                   uint iterations, bool overwrite);

} // namespace EdgeMatching
