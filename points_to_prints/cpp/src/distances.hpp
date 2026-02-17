#pragma once

#include <string>

/**
 * @brief Computes distances between points in order based on GPS Time and
 * Return Number. The distance is the smallest distance to all the points that
 * have the next GPS time in case of multiple returns.
 *
 * @param input_file Input LAS file path
 * @param output_distances_file Output LAS file path containing the initial
 * points with distances
 * @param output_edges_file Output LAS file path containing the generated points
 * to detect edges
 * @param overwrite Whether to overwrite the output file if it already exists
 */
void compute_distances_in_order(const std::string &input_file,
                                const std::string &output_distances_file,
                                const std::string &output_edges_file,
                                bool overwrite);

struct DistancesInOrderOptions {
    std::string input_file;
    std::string output_distances_file;
    std::string output_edges_file;
    bool overwrite;
};
