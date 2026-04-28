#include <string>
#include <sys/types.h>

#include <CLI11/CLI11.hpp>

#include "distances.hpp"
#include "edge_matching/topology.hpp"
#include "footprints/footprints.hpp"
#include "las_filter.hpp"
#include "parquet.hpp"
#include "roofprints/transfer_3d.hpp"

void setup_sort_by_gps_time(CLI::App &app) {
    auto opt = std::make_shared<SortByGpsTimeOptions>();

    CLI::App *sub =
        app.add_subcommand("sort_by_gps_time", "Sort points by GPS time");
    sub->add_option("-i,--input", opt->input_file, "Input LAS file")
        ->required();
    sub->add_option("-o,--output", opt->output_file, "Output LAS file")
        ->required();
    sub->add_flag("-f,--overwrite", opt->overwrite,
                  "Overwrite the output file if it exists")
        ->default_val(false);

    sub->callback([opt]() {
        sort_by_gps_time(opt->input_file, opt->output_file, opt->overwrite);
    });
}

void setup_distances_in_order(CLI::App &app) {
    auto opt = std::make_shared<DistancesInOrderOptions>();

    CLI::App *sub = app.add_subcommand(
        "distances_in_order", "Compute the distances in GPS time order");
    sub->add_option("-i,--input", opt->input_points_file, "Input LAS file")
        ->required();
    sub->add_option("-t,--trajectory", opt->input_trajectory_file,
                    "Input trajectory file")
        ->required();
    sub->add_option("-d,--distances", opt->output_distances_file,
                    "Output LAS file for distances")
        ->required();
    sub->add_option("-e,--edges", opt->output_edges_file,
                    "Output LAS file for points on edges")
        ->required();
    sub->add_flag("-f,--overwrite", opt->overwrite,
                  "Overwrite the output file if it exists")
        ->default_val(false);

    sub->callback([opt]() {
        compute_distances_to_neighbours(
            opt->input_points_file, opt->input_trajectory_file,
            opt->output_distances_file, opt->output_edges_file, opt->overwrite);
    });
}

void setup_extract_random_lines(CLI::App &app) {
    auto opt = std::make_shared<ExtractRandomLinesOptions>();

    CLI::App *sub = app.add_subcommand(
        "extract_random_lines", "Extract random lines from the point cloud");
    sub->add_option("-i,--input", opt->input_file, "Input LAS file")
        ->required();
    sub->add_option("-o,--output", opt->output_folder, "Output folder")
        ->required();
    sub->add_option("-l,--lines", opt->lines_count,
                    "Number of lines to extract")
        ->required();
    sub->add_flag("-f,--overwrite", opt->overwrite,
                  "Overwrite the output folder if it exists")
        ->default_val(false);

    sub->callback([opt]() {
        extract_random_scanner_lines(opt->input_file, opt->output_folder,
                                     opt->lines_count, opt->overwrite);
    });
}

void setup_split_flight_axes(CLI::App &app) {
    auto opt = std::make_shared<ExtractRandomLinesOptions>();

    CLI::App *sub =
        app.add_subcommand("split_flight_axes", "Split the flight axes");
    sub->add_option("-i,--input", opt->input_file, "Input LAS file")
        ->required();
    sub->add_option("-o,--output", opt->output_folder, "Output folder")
        ->required();
    sub->add_flag("-f,--overwrite", opt->overwrite,
                  "Overwrite the output folder if it exists")
        ->default_val(false);

    sub->callback([opt]() {
        split_flight_axes(opt->input_file, opt->output_folder, opt->overwrite);
    });
}

void setup_test_parquet(CLI::App &app) {
    auto opt = std::make_shared<TestParquetOptions>();

    CLI::App *sub =
        app.add_subcommand("test_parquet", "Test Parquet Read/Write");
    sub->add_option("-i,--input", opt->input_file, "Input Parquet file")
        ->required();

    sub->callback([opt]() { return open_parquet(opt->input_file); });
}

void setup_read_write_bd_topo(CLI::App &app) {
    auto opt = std::make_shared<ReadWriteBDTOPOOptions>();

    CLI::App *sub = app.add_subcommand(
        "read_write_bd_topo",
        "Read and write building outlines from/to BD TOPO Parquet file");
    sub->add_option("-i,--input", opt->input_parquet_file,
                    "Input BD TOPO Parquet file")
        ->required();
    sub->add_option("-o,--output", opt->output_parquet_file,
                    "Output Parquet file for building outlines")
        ->required();
    sub->add_flag("-f,--overwrite", opt->overwrite,
                  "Overwrite the output file if it exists")
        ->default_val(false);

    sub->callback([opt]() {
        auto status = read_write_bd_topo(
            opt->input_parquet_file, opt->output_parquet_file, opt->overwrite);
        if (!status.ok()) {
            std::cerr << "Error in read_write_bd_topo: " << status.ToString()
                      << std::endl;
        }
    });
}

void setup_compute_roofprints(CLI::App &app) {
    auto opt = std::make_shared<AllLines::ComputeRoofprintsOptions>();

    CLI::App *sub =
        app.add_subcommand("compute_roofprints", "Compute roofprints");
    sub->add_option("-l,--input-las", opt->input_las_file, "Input LAS file")
        ->required();
    sub->add_option("-e,--input-bd-topo-edges", opt->input_bd_topo_edges_file,
                    "Input BD TOPO Parquet file with building edges")
        ->required();
    sub->add_option("-i,--input-bd-topo-intersections",
                    opt->input_bd_topo_intersections_file,
                    "Input BD TOPO Parquet file with building intersections")
        ->required();
    sub->add_option("-o,--output-roofprints", opt->output_roofprints_file,
                    "Output Parquet file for roofprints")
        ->required();
    sub->add_option("-n,--iterations", opt->iterations,
                    "Number of optimization iterations to perform")
        ->required();
    sub->add_flag("-f,--overwrite", opt->overwrite,
                  "Overwrite the output file if it exists")
        ->default_val(false);

    sub->callback([opt]() {
        AllLines::compute_roofprints(
            opt->input_las_file, opt->input_bd_topo_edges_file,
            opt->input_bd_topo_intersections_file, opt->output_roofprints_file,
            opt->iterations, opt->overwrite);
    });
}

void setup_add_inward_directions(CLI::App &app) {
    auto opt = std::make_shared<InwardDirectionsOptions>();

    CLI::App *sub = app.add_subcommand("add_inward_directions",
                                       "Add inward directions to LAS");

    sub->add_option("-i,--input", opt->input_points_file, "Input LAS file")
        ->required();
    sub->add_option("-o,--output", opt->output_points_file,
                    "Output LAS file for points with inward directions")
        ->required();
    sub->add_option("-t,--type", opt->type,
                    "Type of inward direction to compute: 'roof' or 'facade'")
        ->required();
    sub->add_flag("-f,--overwrite", opt->overwrite,
                  "Overwrite the output file if it exists")
        ->default_val(false);

    sub->callback([opt]() {
        compute_inward_directions(opt->input_points_file,
                                  opt->output_points_file, opt->type,
                                  opt->overwrite);
    });
}

void setup_roofprints_to_3d(CLI::App &app) {
    auto opt = std::make_shared<RooprintsTo3DOptions>();

    CLI::App *sub =
        app.add_subcommand("roofprints_to_3d", "Convert roofprints to 3D");
    sub->add_option("-i,--input", opt->input_roofprints_file,
                    "Input Parquet file with roofprints")
        ->required();
    sub->add_option("-e,--edge-points", opt->edge_points_file,
                    "Input LAS file with edge points")
        ->required();
    sub->add_option("-o,--output", opt->output_roofprints_3d_file,
                    "Output Parquet file for 3D roofprints")
        ->required();
    sub->add_flag("-f,--overwrite", opt->overwrite,
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

void setup_roofprints_3d_to_footprints(CLI::App &app) {
    auto opt = std::make_shared<Roofprints3DToFootprintsOptions>();

    CLI::App *sub =
        app.add_subcommand("roofprints_3d_to_footprints",
                           "Convert 3D roofprints to 2D footprints");
    sub->add_option("-i,--input", opt->input_roofprints_file,
                    "Input Parquet file with 3D roofprints")
        ->required();
    sub->add_option("-p,--points", opt->points_file,
                    "Input LAS file with points for footprint computation")
        ->required();
    sub->add_option("-t,--trajectory", opt->trajectory_file,
                    "Input trajectory file for footprint computation")
        ->required();
    sub->add_option("-f,--output-footprints", opt->output_footprints_file,
                    "Output Parquet file for 2D footprints")
        ->required();
    sub->add_option("-o,--output-points", opt->output_points_file,
                    "Output LAS file for points with corresponding building ID")
        ->required();
    sub->add_flag("--overwrite", opt->overwrite,
                  "Overwrite the output file if it exists")
        ->default_val(false);

    sub->callback([opt]() {
        auto status = roofprints_3d_to_footprints(
            opt->input_roofprints_file, opt->points_file, opt->trajectory_file,
            opt->output_footprints_file, opt->output_points_file,
            opt->overwrite);
        if (!status.ok()) {
            std::cerr << "Error in roofprints_3d_to_footprints: "
                      << status.ToString() << std::endl;
        }
    });
}

struct HelloWorldOptions {
    std::string name;
};

void setup_add_hello_world(CLI::App &app) {
    auto opt = std::make_shared<HelloWorldOptions>();

    CLI::App *sub = app.add_subcommand("hello_world", "Print Hello World");
    sub->add_option("-n,--name", opt->name, "Name to greet")->required();
    sub->callback(
        [opt]() { std::cout << "Hello, " << opt->name << "!" << std::endl; });
}

int main(int argc, char **argv) {
    CLI::App app{"Roofprint and Footprint Extraction"};

    setup_sort_by_gps_time(app);
    setup_split_flight_axes(app);
    setup_extract_random_lines(app);
    setup_distances_in_order(app);
    setup_read_write_bd_topo(app);
    setup_compute_roofprints(app);
    setup_add_inward_directions(app);
    setup_roofprints_to_3d(app);
    setup_roofprints_3d_to_footprints(app);
    setup_test_parquet(app);
    setup_add_hello_world(app);
    app.require_subcommand(1);

    CLI11_PARSE(app, argc, argv);

    return 0;
}
