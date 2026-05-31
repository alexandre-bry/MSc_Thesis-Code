#pragma once

#include <string>

#include "../edge_matching/topology.hpp"

namespace EdgeMatching {

class AllFootprints : public AllOutlines {
  protected:
    void
    compute_weights(PtsStructs::StoragePtr las_points,
                    const std::vector<PtsStructs::PointId> &point_ids,
                    std::vector<double> &weights,
                    std::vector<Vector_2> &point_inward_dirs) const override;

    std::unique_ptr<ICriterion>
    create_criterion(std::vector<Point_2> points, std::vector<double> weights,
                     std::vector<Vector_2> point_inward_dirs) const override;

  public:
    AllFootprints(const EdgeVector<Edge> &edges,
                  const OutlineVector<OutlineAsEdges> &outlines,
                  const std::vector<std::pair<EdgeId, EdgeId>> &intersections)
        : AllOutlines(edges, outlines, intersections) {}

    void prepare_offsets(std::vector<double> &offsets) const override;
};

struct ComputeFootprintsOptions {
    std::string input_las_file;
    std::string input_bd_topo_edges_file;
    std::string input_bd_topo_intersections_file;
    std::string output_roofprints_file;
    uint iterations;
    bool overwrite;
};

void compute_roofprints(const std::string &input_las_file,
                        const std::string &input_bd_topo_edges_file,
                        const std::string &input_bd_topo_intersections_file,
                        const std::string &output_roofprints_file,
                        uint iterations, bool overwrite);

} // namespace EdgeMatching
