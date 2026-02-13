#include <cstdint>
#include <string>

#include <CLI11/CLI11.hpp>

#include "distances.hpp"
#include "parquet.hpp"

void setup_distances_in_order(CLI::App &app) {
    auto opt = std::make_shared<DistancesInOrderOptions>();

    CLI::App *sub = app.add_subcommand(
        "distances_in_order", "Compute the distances in GPS time order");
    std::string input_file;
    sub->add_option("-i,--input", opt->input_file, "Input LAS file")
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
        compute_distances_in_order(opt->input_file, opt->output_distances_file,
                                   opt->output_edges_file, opt->overwrite);
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

void setup_test_parquet(CLI::App &app) {
    auto opt = std::make_shared<TestParquetOptions>();

    CLI::App *sub =
        app.add_subcommand("test_parquet", "Test Parquet Read/Write");
    std::string input_file;
    sub->add_option("-i,--input", opt->input_file, "Input Parquet file")
        ->required();

    sub->callback([opt]() { return open_parquet(opt->input_file); });
}

int main(int argc, char **argv) {
    CLI::App app{"Roofprint and Footprint Extraction"};

    setup_distances_in_order(app);
    setup_extract_random_lines(app);
    setup_test_parquet(app);

    app.require_subcommand(1);

    CLI11_PARSE(app, argc, argv);

    return 0;
}
