#pragma once

#include <random>
#include <vector>

#include "utils/cgal.hpp"

struct RansacLine {
    Line_3 line;
    std::vector<std::size_t> point_indices;
    double score;
};

struct RANSAC3DFitter {
  private:
    const std::vector<Point_3> &points;
    int max_iterations;
    double radius_for_second_point;
    double distance_threshold;
    int max_icp_iterations;

    std::random_device rd;
    std::mt19937 gen;

    // Track which points have been assigned to lines
    std::vector<bool> assigned;

    /**
     * @brief Get a random unassigned point index
     * @return Index of a random unassigned point, or -1 if none available
     */
    int get_random_unassigned_point();

    /**
     * @brief Find all unassigned points within a given radius from a point
     * @param point_idx Index of the center point
     * @param radius Search radius
     * @return Vector of indices of points within radius
     */
    std::vector<std::size_t> get_points_in_radius(std::size_t point_idx,
                                                  double radius);

    /**
     * @brief Find all unassigned points close to a 3D line
     * @param line The 3D line
     * @return Vector of indices of points close to the line
     */
    std::vector<std::size_t> get_points_close_to_line(const Line_3 &line);

    /**
     * @brief Optimize a line using an ICP-like iterative process
     * @param line Initial line to optimize
     * @param point_indices Indices of points to use for optimization
     * @return Optimized line
     */
    Line_3
    optimize_line_with_icp(const Line_3 &line,
                           const std::vector<std::size_t> &point_indices);

  public:
    /**
     * @brief Constructor for RANSAC 3D line fitter
     * @param points Vector of 3D points to fit
     * @param max_iterations Maximum number of iterations
     * @param radius_for_second_point Radius for selecting second point
     * (default 5.0)
     * @param distance_threshold Distance threshold for point-to-line membership
     * (default 0.5)
     * @param max_icp_iterations Maximum number of ICP iterations for line
     * optimization (default 10)
     */
    RANSAC3DFitter(const std::vector<Point_3> &points, int max_iterations,
                   double radius_for_second_point = 5.0,
                   double distance_threshold = 0.5,
                   int max_icp_iterations = 10);

    /**
     * @brief Run the RANSAC algorithm
     * @return Vector of fitted lines with their associated points
     */
    std::vector<RansacLine> run();

    /**
     * @brief Extract a finite segment from a RansacLine by finding the extent
     * of points along the line
     * @param ransac_line The RansacLine containing infinite line and point
     * indices
     * @return A finite 3D segment spanning the extent of the points
     */
    Segment_3 extract_segment(const RansacLine &ransac_line) const;
};
