#pragma once

#include <string>

/**
 * @brief Computes distances between points in order based on GPS Time and
 * Return Number. The distance is the smallest distance to all the points that
 * have the next GPS time in case of multiple returns.
 *
 * @param input_file Input LAS file path
 * @param input_trajectory_file Input LAS file path containing the trajectory of
 * the scanner device
 * @param output_distances_file Output LAS file path containing the initial
 * points with distances
 * @param output_edges_file Output LAS file path containing the generated points
 * to detect edges
 * @param overwrite Whether to overwrite the output file if it already exists
 */
void compute_distances_in_order(const std::string &input_points_file,
                                const std::string &input_trajectory_file,
                                const std::string &output_distances_file,
                                const std::string &output_edges_file,
                                bool overwrite);

struct DistancesInOrderOptions {
    std::string input_points_file;
    std::string input_trajectory_file;
    std::string output_distances_file;
    std::string output_edges_file;
    bool overwrite;
};

/**
 * @brief Computes the inward direction for each point based on the local
 * neighborhood of points. The expected meaning of the inward direction is the
 * direction towards the inside of the building, hopefully perpendicularly to
 * the edge.
 *
 * @param input_points_file Input LAS file path
 * @param input_trajectory_file Input LAS file path containing the trajectory of
 * the scanner device
 * @param output_points_file Output LAS file path containing the points with
 * inward directions
 * @param overwrite Whether to overwrite the output file if it already exists
 */
void compute_inward_directions(const std::string &input_points_file,
                               const std::string &output_points_file,
                               bool overwrite);

struct InwardDirectionsOptions {
    std::string input_points_file;
    std::string output_points_file;
    bool overwrite;
};