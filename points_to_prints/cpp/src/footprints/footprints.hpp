#include <string>

struct ComputeFootprintsOptions {
    std::string input_las_file;
    std::string input_bd_topo_edges_file;
    std::string input_bd_topo_intersections_file;
    std::string output_footprints_file;
    uint iterations;
    bool overwrite;
};

void compute_footprints(const std::string &input_las_file,
                        const std::string &input_bd_topo_edges_file,
                        const std::string &input_bd_topo_intersections_file,
                        const std::string &output_footprints_file,
                        uint iterations, bool overwrite);