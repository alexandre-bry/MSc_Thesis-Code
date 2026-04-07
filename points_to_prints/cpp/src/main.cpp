#include <cstdint>
#include <string>

#include <CLI11/CLI11.hpp>

#include "distances.hpp"
#include "edge_matching/topology.hpp"
#include "las_filter.hpp"
#include "parquet.hpp"
#include "pca.hpp"

void setup_sort_by_gps_time(CLI::App &app) {
    auto opt = std::make_shared<SortByGpsTimeOptions>();

    CLI::App *sub =
        app.add_subcommand("sort_by_gps_time", "Sort points by GPS time");
    std::string input_file;
    sub->add_option("-i,--input", opt->input_file, "Input LAS file")
        ->required();
    std::string output_file;
    sub->add_option("-o,--output", opt->output_file, "Output LAS file")
        ->required();
    bool overwrite = false;
    sub->add_flag("-f,--overwrite", opt->overwrite,
                  "Overwrite the output file if it exists");

    sub->callback([opt]() {
        sort_by_gps_time(opt->input_file, opt->output_file, opt->overwrite);
    });
}

void setup_distances_in_order(CLI::App &app) {
    auto opt = std::make_shared<DistancesInOrderOptions>();

    CLI::App *sub = app.add_subcommand(
        "distances_in_order", "Compute the distances in GPS time order");
    std::string input_points_file;
    sub->add_option("-i,--input", opt->input_points_file, "Input LAS file")
        ->required();
    std::string input_trajectory_file;
    sub->add_option("-t,--trajectory", opt->input_trajectory_file,
                    "Input trajectory file")
        ->required();
    std::string output_distances_file;
    sub->add_option("-d,--distances", opt->output_distances_file,
                    "Output LAS file for distances")
        ->required();
    std::string output_edges_file;
    sub->add_option("-e,--edges", opt->output_edges_file,
                    "Output LAS file for points on edges")
        ->required();
    bool overwrite = false;
    sub->add_flag("-f,--overwrite", opt->overwrite,
                  "Overwrite the output file if it exists");

    sub->callback([opt]() {
        compute_distances_in_order(
            opt->input_points_file, opt->input_trajectory_file,
            opt->output_distances_file, opt->output_edges_file, opt->overwrite);
    });
}

void setup_extract_random_lines(CLI::App &app) {
    auto opt = std::make_shared<ExtractRandomLinesOptions>();

    CLI::App *sub = app.add_subcommand(
        "extract_random_lines", "Extract random lines from the point cloud");
    std::string input_file;
    sub->add_option("-i,--input", opt->input_file, "Input LAS file")
        ->required();
    std::string output_folder;
    sub->add_option("-o,--output", opt->output_folder, "Output folder")
        ->required();
    uint16_t lines_count;
    sub->add_option("-l,--lines", opt->lines_count,
                    "Number of lines to extract")
        ->required();
    bool overwrite = false;
    sub->add_flag("-f,--overwrite", opt->overwrite,
                  "Overwrite the output folder if it exists");

    sub->callback([opt]() {
        extract_random_scanner_lines(opt->input_file, opt->output_folder,
                                     opt->lines_count, opt->overwrite);
    });
}

void setup_split_flight_axes(CLI::App &app) {
    auto opt = std::make_shared<ExtractRandomLinesOptions>();

    CLI::App *sub =
        app.add_subcommand("split_flight_axes", "Split the flight axes");
    std::string input_file;
    sub->add_option("-i,--input", opt->input_file, "Input LAS file")
        ->required();
    std::string output_folder;
    sub->add_option("-o,--output", opt->output_folder, "Output folder")
        ->required();
    bool overwrite = false;
    sub->add_flag("-f,--overwrite", opt->overwrite,
                  "Overwrite the output folder if it exists");

    sub->callback([opt]() {
        split_flight_axes(opt->input_file, opt->output_folder, opt->overwrite);
    });
}

void setup_test_parquet(CLI::App &app) {
    auto opt = std::make_shared<TestParquetOptions>();

    CLI::App *sub =
        app.add_subcommand("test_parquet", "Test Parquet Read/Write");
    std::string input_file;
    sub->add_option("-i,--input", opt->input_file, "Input Parquet file")
        ->required();

    sub->callback([opt]() { return open_parquet(opt->input_file); });
}

void setup_read_write_bd_topo(CLI::App &app) {
    auto opt = std::make_shared<ReadWriteBDTOPOOptions>();

    CLI::App *sub = app.add_subcommand(
        "read_write_bd_topo",
        "Read and write building outlines from/to BD TOPO Parquet file");
    std::string input_file;
    sub->add_option("-i,--input", opt->input_parquet_file,
                    "Input BD TOPO Parquet file")
        ->required();
    std::string output_file;
    sub->add_option("-o,--output", opt->output_parquet_file,
                    "Output Parquet file for building outlines")
        ->required();
    bool overwrite = false;
    sub->add_flag("-f,--overwrite", opt->overwrite,
                  "Overwrite the output file if it exists");

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
    std::string input_las_file;
    sub->add_option("-l,--input-las", opt->input_las_file, "Input LAS file")
        ->required();
    std::string input_bd_topo_edges_file;
    sub->add_option("-b,--input-bd-topo-edges", opt->input_bd_topo_edges_file,
                    "Input BD TOPO Parquet file with building edges")
        ->required();
    std::string input_bd_topo_intersections_file;
    sub->add_option("-i,--input-bd-topo-intersections",
                    opt->input_bd_topo_intersections_file,
                    "Input BD TOPO Parquet file with building intersections")
        ->required();
    std::string output_roofprints_file;
    sub->add_option("-o,--output-roofprints", opt->output_roofprints_file,
                    "Output Parquet file for roofprints")
        ->required();
    bool overwrite = false;
    sub->add_flag("-f,--overwrite", opt->overwrite,
                  "Overwrite the output file if it exists");

    sub->callback([opt]() {
        AllLines::compute_roofprints(
            opt->input_las_file, opt->input_bd_topo_edges_file,
            opt->input_bd_topo_intersections_file, opt->output_roofprints_file,
            opt->overwrite);
    });
}

void setup_add_pca(CLI::App &app) {
    auto opt = std::make_shared<AddPCAOptions>();

    CLI::App *sub = app.add_subcommand("add_pca", "Add PCA eigenvalues to LAS");
    std::string input_file;
    sub->add_option("-i,--input", opt->input_file, "Input LAS file")
        ->required();
    std::string output_file;
    sub->add_option("-o,--output", opt->output_file, "Output LAS file")
        ->required();
    bool overwrite = false;
    sub->add_flag("-f,--overwrite", opt->overwrite,
                  "Overwrite the output file if it exists");

    sub->callback([opt]() {
        add_pca(opt->input_file, opt->output_file, opt->overwrite);
    });
}

int main(int argc, char **argv) {
    CLI::App app{"Roofprint and Footprint Extraction"};

    setup_sort_by_gps_time(app);
    setup_split_flight_axes(app);
    setup_extract_random_lines(app);
    setup_distances_in_order(app);
    setup_read_write_bd_topo(app);
    setup_test_parquet(app);
    setup_compute_roofprints(app);
    setup_add_pca(app);
    app.require_subcommand(1);

    CLI11_PARSE(app, argc, argv);

    return 0;
}
