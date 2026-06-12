#include <string>
#include <sys/types.h>

#include <CLI11/CLI11.hpp>

#include "footprints/footprints.hpp"
#include "footprints/points_selection.hpp"
#include "roofprint_to_3d/transfer_3d.hpp"
#include "roofprints/distances.hpp"
#include "roofprints/roofprints.hpp"

void setup_identify_roof_edge_points(CLI::App &app) {
    auto opt = std::make_shared<IdentifyRoofEdgePointsOptions>();

    CLI::App *sub = app.add_subcommand(
        "roof_edge_points", "Identify roof edge points in the point cloud");
    sub->add_option("-i,--input", opt->input_points_file, "Input LAS/LAZ file")
        ->required();
    sub->add_option("-t,--trajectory", opt->input_trajectory_file,
                    "Input trajectory file")
        ->required();
    sub->add_option("-d,--distances", opt->output_distances_file,
                    "Output LAS/LAZ file for distances")
        ->required();
    sub->add_option("-e,--edges", opt->output_edges_file,
                    "Output LAS/LAZ file for points on edges")
        ->required();
    sub->add_flag("--overwrite", opt->overwrite,
                  "Overwrite the output file if it exists")
        ->default_val(false);

    sub->callback([opt]() {
        identify_roof_edge_points(
            opt->input_points_file, opt->input_trajectory_file,
            opt->output_distances_file, opt->output_edges_file, opt->overwrite);
    });
}

void setup_compute_roofprints(CLI::App &app) {
    auto opt = std::make_shared<EdgeMatching::ComputeRoofprintsOptions>();

    CLI::App *sub = app.add_subcommand(
        "roofprints", "Compute the roofprints from the roof edge points and "
                      "the building edges and intersections from BD TOPO");
    sub->add_option("-l,--input-las", opt->input_las_file,
                    "Input LAS/LAZ file containing the roof edge points")
        ->required();
    sub->add_option("-e,--input-bd-topo-edges", opt->input_bd_topo_edges_file,
                    "Input BD TOPO Parquet file with building edges")
        ->required();
    sub->add_option("-i,--input-bd-topo-intersections",
                    opt->input_bd_topo_intersections_file,
                    "Input BD TOPO Parquet file with building intersections")
        ->required();
    sub->add_option(
           "-o,--output-roofprints-template",
           opt->output_roofprints_template_file,
           "Output Parquet template file for roofprints which must contain "
           "{iteration} as a substring to be replaced by the iteration number")
        ->required();
    sub->add_option("-n,--iterations", opt->iterations,
                    "Number of optimization iterations to perform")
        ->required();
    sub->add_flag("--overwrite", opt->overwrite,
                  "Overwrite the output file if it exists")
        ->default_val(false);

    sub->callback([opt]() {
        auto status = EdgeMatching::compute_roofprints(
            opt->input_las_file, opt->input_bd_topo_edges_file,
            opt->input_bd_topo_intersections_file,
            opt->output_roofprints_template_file, opt->iterations,
            opt->overwrite);
        if (!status.ok()) {
            std::cerr << "Error in compute_roofprints: " << status.ToString()
                      << std::endl;
        }
    });
}

void setup_add_inward_directions(CLI::App &app) {
    auto opt = std::make_shared<InwardDirectionsOptions>();

    CLI::App *sub = app.add_subcommand(
        "inward_directions",
        "Compute the inward direction for each point in the point cloud");

    sub->add_option("-i,--input", opt->input_points_file, "Input LAS/LAZ file")
        ->required();
    sub->add_option("-o,--output", opt->output_points_file,
                    "Output LAS/LAZ file for points with inward directions")
        ->required();
    sub->add_option("-t,--type", opt->type,
                    "Type of inward direction to compute: 'roof' or 'facade'")
        ->required();
    sub->add_flag("--overwrite", opt->overwrite,
                  "Overwrite the output file if it exists")
        ->default_val(false);

    sub->callback([opt]() {
        compute_inward_directions(opt->input_points_file,
                                  opt->output_points_file, opt->type,
                                  opt->overwrite);
    });
}

void setup_compute_footprints(CLI::App &app) {
    auto opt = std::make_shared<EdgeMatching::ComputeFootprintsOptions>();

    CLI::App *sub = app.add_subcommand(
        "footprints", "Compute the footprints from the roofprints, the LoD2.2 "
                      "building models and the point cloud");
    sub->add_option("-p,--input-points", opt->input_points_file,
                    "Input LAS/LAZ file")
        ->required();
    sub->add_option("-l,--input-lod22", opt->input_lod22_file,
                    "Input Parquet file with LoD2.2 data")
        ->required();
    sub->add_option("-r,--input-roofprints", opt->input_roofprints_file,
                    "Input Parquet file with roofprints")
        ->required();
    sub->add_option("-o,--output-footprints-template",
                    opt->output_footprints_template_file,
                    "Output Parquet template file for footprints which must "
                    "contain {iteration} as a substring to be replaced by the "
                    "iteration number")
        ->required();
    sub->add_option("-n,--iterations", opt->iterations,
                    "Number of optimization iterations to perform")
        ->required();
    sub->add_flag("--overwrite", opt->overwrite,
                  "Overwrite the output file if it exists")
        ->default_val(false);

    sub->callback([opt]() {
        auto status = EdgeMatching::compute_footprints(
            opt->input_roofprints_file, opt->input_lod22_file,
            opt->input_points_file, opt->output_footprints_template_file,
            opt->iterations, opt->overwrite);
        if (!status.ok()) {
            std::cerr << "Error in compute_footprints: " << status.ToString()
                      << std::endl;
        }
    });
}

void setup_roofprints_to_3d(CLI::App &app) {
    auto opt = std::make_shared<RooprintsTo3DOptions>();

    CLI::App *sub = app.add_subcommand(
        "roofprints_to_3d",
        "(Experimental) Move the edges of the roofprints to 3D by finding 3D "
        "segments in their vertical planes");
    sub->add_option("-i,--input", opt->input_roofprints_file,
                    "Input Parquet file with roofprints")
        ->required();
    sub->add_option("-e,--edge-points", opt->edge_points_file,
                    "Input LAS/LAZ file with edge points")
        ->required();
    sub->add_option("-o,--output", opt->output_roofprints_3d_file,
                    "Output Parquet file for 3D roofprints")
        ->required();
    sub->add_flag("--overwrite", opt->overwrite,
                  "Overwrite the output file if it exists")
        ->default_val(false);

    sub->callback([opt]() {
        auto status =
            roofprints_to_3d(opt->input_roofprints_file, opt->edge_points_file,
                             opt->output_roofprints_3d_file, opt->overwrite);
        if (!status.ok()) {
            std::cerr << "Error in roofprints_to_3d: " << status.ToString()
                      << std::endl;
        }
    });
}

void setup_select_points_under_roofs(CLI::App &app) {
    auto opt =
        std::make_shared<PointSelection::SelectPointsUnderRoofsOptions>();

    CLI::App *sub = app.add_subcommand(
        "select_points_under_roofs",
        "Keep only the points which are under at least one roof surface, with "
        "a certain vertical and horizontal tolerance");
    sub->add_option("-p,--input-points", opt->input_points_file,
                    "Input LAS/LAZ file")
        ->required();
    sub->add_option("-l,--input-lod22", opt->input_roofs_file,
                    "Input CityJSON file with building roofs")
        ->required();
    sub->add_option("-o,--output-points", opt->output_points_file,
                    "Output LAS/LAZ file with selected points")
        ->required();
    sub->add_option(
           "-v,--vertical-buffer", opt->vertical_buffer,
           "Vertical tolerance below roof surfaces, pointing downwards")
        ->default_val(0.5);
    sub->add_option(
           "-x,--horizontal-buffer", opt->horizontal_buffer,
           "Horizontal buffer around roof boundaries, pointing outwards")
        ->default_val(0.3);
    sub->add_flag("--overwrite", opt->overwrite,
                  "Overwrite the output file if it exists")
        ->default_val(false);

    sub->callback([opt]() {
        PointSelection::select_points_under_roofs(
            opt->input_points_file, opt->input_roofs_file,
            opt->output_points_file, opt->vertical_buffer,
            opt->horizontal_buffer, opt->overwrite);
    });
}

struct HelloWorldOptions {
    std::string name;
};

CLI::App &setup_add_hello_world(CLI::App &app) {
    auto opt = std::make_shared<HelloWorldOptions>();

    CLI::App *sub = app.add_subcommand("hello_world", "Print Hello World");
    sub->add_option("-n,--name", opt->name, "Name to greet")->required();
    sub->callback(
        [opt]() { std::cout << "Hello, " << opt->name << "!" << std::endl; });

    return *sub;
}

int main(int argc, char **argv) {
    CLI::App app{"Roofprint and Footprint Extraction"};
    app.require_subcommand(1);

    // Main commands
    setup_add_inward_directions(app);
    setup_identify_roof_edge_points(app);
    setup_compute_roofprints(app);
    setup_compute_footprints(app);

    CLI::App *sub_utils_roof = app.add_subcommand(
        "utils_roof", "Subcommands related to roofprints and roofs");

    // Utility commands for roofprints and roofs
    setup_roofprints_to_3d(*sub_utils_roof);
    setup_select_points_under_roofs(*sub_utils_roof);

    auto &hello_world_subcommand = setup_add_hello_world(app);

    // Parse the command line arguments and execute the corresponding command
    CLI11_PARSE(app, argc, argv);

    return 0;
}
