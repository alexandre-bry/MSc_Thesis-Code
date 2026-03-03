#include "las_filter.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <random>
#include <string>
#include <unordered_map>

#include <pdal/Dimension.hpp>
#include <pdal/pdal_types.hpp>

#include "las/reader.hpp"
#include "las/writer.hpp"
#include "pbar.hpp"
#include "points.hpp"

void extract_random_scanner_lines(const std::string &input_file,
                                  const std::string &output_folder,
                                  uint16_t lines_count, bool overwrite) {
    if (std::filesystem::exists(output_folder) && !overwrite) {
        throw std::runtime_error("Output folder already exists: " +
                                 output_folder);
    }

    // Create the output folder
    std::filesystem::create_directories(output_folder);

    // Read the LAS file and get the point view
    std::cout << "Reading LAS file..." << std::endl;
    CustomLasReader las_reader(input_file);
    las_reader.execute();
    auto in_view = las_reader.point_view();
    auto [predefined_dims, proprietary_dims] = las_reader.dimensions();
    auto n_features = las_reader.point_count();

    std::cout << "Number of points: " << n_features << std::endl;

    // Prepare the points with attributes
    Points3DAttrGPSSorted points(in_view);

    std::cout << "Preparing output objects..." << std::endl;

    // Prepare the random number generator
    std::mt19937 rng;
    std::uniform_int_distribution<std::size_t> uint_dist(0, n_features - 1);

    // Prepare the progress bar
    ProgressBarTotal bar(
        lines_count, "Extracting random scanner lines",
        indicators::option::ForegroundColor{indicators::Color::green});
    bar.start();

    for (auto line_number = 0; line_number < lines_count; line_number++) {
        // Select a random point in the file
        std::size_t idx0 = uint_dist(rng);
        Point3DWithAttributes p0 = points[idx0];

        // Select all the points in the same line
        std::vector<std::size_t> line_idxs({idx0});
        std::vector<std::size_t> prev_idxs, next_idxs;

        prev_idxs = {idx0};
        next_idxs = points.get_indices_for_next_gps_time(p0.gps_time);
        while (points.any_neighbours(prev_idxs, next_idxs)) {
            line_idxs.insert(line_idxs.end(), next_idxs.begin(),
                             next_idxs.end());
            prev_idxs = next_idxs;
            if (next_idxs.empty()) {
                break;
            }
            next_idxs = points.get_indices_for_next_gps_time(
                points[next_idxs[0]].gps_time);
        }

        prev_idxs = {idx0};
        next_idxs = points.get_indices_for_prev_gps_time(p0.gps_time);
        while (points.any_neighbours(prev_idxs, next_idxs)) {
            line_idxs.insert(line_idxs.end(), next_idxs.begin(),
                             next_idxs.end());
            prev_idxs = next_idxs;
            if (next_idxs.empty()) {
                break;
            }
            next_idxs = points.get_indices_for_prev_gps_time(
                points[next_idxs[0]].gps_time);
        }

        // Initialize the writer
        CustomLasWriter las_writer(predefined_dims, proprietary_dims,
                                   las_reader.table.spatialReference());

        // Add all the points to the writer
        for (std::size_t out_idx = 0; out_idx < line_idxs.size(); out_idx++) {
            std::size_t initial_idx = line_idxs[out_idx];
            auto sorted_idx = points.to_sorted_index(initial_idx);
            for (pdal::Dimension::Id dim : predefined_dims) {
                auto value = las_reader.getFieldAs<double>(dim, initial_idx);
                las_writer.setField(dim, out_idx, value);
            }
            for (auto dim : proprietary_dims) {
                auto value = las_reader.getFieldAs<double>(dim, initial_idx);
                las_writer.setField(dim, out_idx, value);
            }
        }

        std::string output_file =
            output_folder + "/line_" + std::to_string(line_number) + ".laz";
        las_writer.write(output_file, {});

        bar.increment(1);
    }
    bar.finish();
    std::cout << "Done." << std::endl;
}

void split_flight_axes(const std::string &input_file,
                       const std::string &output_folder, bool overwrite) {
    if (std::filesystem::exists(output_folder) && !overwrite) {
        throw std::runtime_error("Output folder already exists: " +
                                 output_folder);
    }

    // Create the output folder
    std::filesystem::create_directories(output_folder);

    // Read the LAS file and get the point view
    std::cout << "Reading LAS file..." << std::endl;
    CustomLasReader las_reader(input_file);
    las_reader.execute();
    auto in_view = las_reader.point_view();
    auto [predefined_dims, proprietary_dims] = las_reader.dimensions();
    auto n_features = las_reader.point_count();

    std::cout << "Number of points: " << n_features << std::endl;

    std::cout << "Preparing output objects..." << std::endl;

    // Prepare the progress bar
    ProgressBarTotal bar(
        n_features, "Splitting flight axes",
        indicators::option::ForegroundColor{indicators::Color::green});
    bar.start();

    // Prepare the points with attributes
    Points3DAttrGPSSorted points(in_view);

    // Initialize the map of writers
    std::unordered_map<int, CustomLasWriter *> source_id_to_writers;

    for (auto initial_idx = 0; initial_idx < n_features; initial_idx++) {
        // Get the point and its attributes
        Point3DWithAttributes p = points[initial_idx];

        // Get the writer for the point source ID, or create it if it doesn't
        // exist
        if (source_id_to_writers.find(p.point_source_id) ==
            source_id_to_writers.end()) {
            CustomLasWriter *las_writer =
                new CustomLasWriter(predefined_dims, proprietary_dims,
                                    las_reader.table.spatialReference());
            source_id_to_writers[p.point_source_id] = las_writer;
        }
        CustomLasWriter *las_writer = source_id_to_writers[p.point_source_id];

        // Add this point to the writer
        auto out_idx = las_writer->pointCount();
        for (pdal::Dimension::Id dim : predefined_dims) {
            auto value = las_reader.getFieldAs<double>(dim, initial_idx);
            las_writer->setField(dim, out_idx, value);
        }
        for (auto dim : proprietary_dims) {
            auto value = las_reader.getFieldAs<double>(dim, initial_idx);
            las_writer->setField(dim, out_idx, value);
        }

        bar.increment(1);
    }
    bar.finish();

    // Write the output files
    for (const auto &[source_id, las_writer] : source_id_to_writers) {
        std::string output_file =
            output_folder + "/axis_" + std::to_string(source_id) + ".laz";
        las_writer->write(output_file, {});
    }

    std::cout << "Done." << std::endl;
}

void sort_by_gps_time(const std::string &input_file,
                      const std::string &output_file, bool overwrite) {
    if (std::filesystem::exists(output_file) && !overwrite) {
        throw std::runtime_error("Output file already exists: " + output_file);
    }

    // Read the LAS file and get the point view
    std::cout << "Reading LAS file..." << std::endl;
    CustomLasReader las_reader(input_file);
    las_reader.execute();
    auto in_view = las_reader.point_view();
    auto [predefined_dims, proprietary_dims] = las_reader.dimensions();
    auto n_features = las_reader.point_count();

    std::cout << "Number of points: " << n_features << std::endl;

    std::cout << "Preparing output objects..." << std::endl;

    // Prepare the progress bar
    ProgressBarTotal bar(
        n_features, "Sorting by GPS time",
        indicators::option::ForegroundColor{indicators::Color::green});
    bar.start();

    // Prepare the points with attributes
    Points3DAttrGPSSorted points(in_view);

    // Initialize writer
    CustomLasWriter *las_writer = new CustomLasWriter(
        predefined_dims, proprietary_dims, las_reader.table.spatialReference());

    for (auto initial_idx_group : points.get_groups_in_gps_time_order()) {
        for (auto initial_idx : initial_idx_group) {
            // Get the point and its attributes
            Point3DWithAttributes p = points[initial_idx];

            // Add this point to the writer
            auto out_idx = las_writer->pointCount();
            for (pdal::Dimension::Id dim : predefined_dims) {
                auto value = las_reader.getFieldAs<double>(dim, initial_idx);
                las_writer->setField(dim, out_idx, value);
            }
            for (auto dim : proprietary_dims) {
                auto value = las_reader.getFieldAs<double>(dim, initial_idx);
                las_writer->setField(dim, out_idx, value);
            }
        }

        bar.increment(1);
    }
    bar.finish();

    // Write the output files
    las_writer->write(output_file, {});

    std::cout << "Done." << std::endl;
}