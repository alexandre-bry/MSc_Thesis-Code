#include "distances.hpp"

#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <string>
#include <type_traits>

#include <pdal/Dimension.hpp>
#include <pdal/pdal_types.hpp>

#include "las.hpp"
#include "pbar.hpp"
#include "points.hpp"

const double MIN_VERT_GAP_ROOF = 1.0;
const double MAX_VERT_GAP_FOOT = -1.0;

void compute_distances_in_order(const std::string &input_file,
                                const std::string &output_distances_file,
                                const std::string &output_edges_file,
                                bool overwrite) {

    if (std::filesystem::exists(output_distances_file) && !overwrite) {
        throw std::runtime_error("Output file already exists: " +
                                 output_distances_file);
    } else if (std::filesystem::exists(output_edges_file) && !overwrite) {
        throw std::runtime_error("Output file already exists: " +
                                 output_edges_file);
    }

    // Read the LAS file and get the point view
    std::cout << "Reading LAS file..." << std::endl;
    CustomLasReader las_reader(input_file);
    las_reader.execute();
    auto in_view = las_reader.point_view();
    auto [predefined_dims, proprietary_dims] = las_reader.dimensions();
    auto n_features = las_reader.point_count();

    std::cout << "Number of points: " << n_features << std::endl;

    // Prepare the points with attributes
    Points3DWithAttributes points(in_view);

    std::cout << "Preparing output objects..." << std::endl;
    std::vector<pdal::Dimension::Id> distances_dims = predefined_dims;
    std::vector<ProprietaryDimension> distances_custom_dims = proprietary_dims;
    std::vector<ProprietaryDimension> new_distances_custom_dims = {
        CustomDimensions::Id::DownSignedVertGap,
        CustomDimensions::Id::UpSignedVertGap,
        CustomDimensions::Id::IsRoofEdge,
        CustomDimensions::Id::IsFootEdge,
    };
    distances_custom_dims.insert(distances_custom_dims.end(),
                                 new_distances_custom_dims.begin(),
                                 new_distances_custom_dims.end());
    CustomLasWriter las_distances_writer(distances_dims, distances_custom_dims,
                                         las_reader.table.spatialReference());

    std::vector<pdal::Dimension::Id> edge_dims = {
        pdal::Dimension::Id::X,
        pdal::Dimension::Id::Y,
        pdal::Dimension::Id::Z,
        pdal::Dimension::Id::Classification,
    };
    std::vector<ProprietaryDimension> edge_custom_dims = {
        CustomDimensions::Id::IsGenerated};
    CustomLasWriter las_edge_writer(edge_dims, edge_custom_dims,
                                    las_reader.table.spatialReference());

    std::cout << "Adding existing dimensions to output view..." << std::endl;
    // Add the existing dimensions to the output point view
    // The points need to be processed in the order of the output view
    for (pdal::PointId out_idx = 0; out_idx < n_features; ++out_idx) {
        std::size_t in_idx = points.to_initial_index(out_idx);
        for (pdal::Dimension::Id dim : predefined_dims) {
            double value = las_reader.getFieldAs<double>(dim, in_idx);
            las_distances_writer.setField(dim, out_idx, value);
            // Get the value of the dimension in the correct type and set it in
            // the output view
            // char *value;
            // in_view->getField(value, dim, pdal::Dimension::defaultType(dim),
            //                   in_idx);
            // las_distances_writer.setField(dim, out_idx, value);
        }
        for (auto dim : proprietary_dims) {
            auto value = las_reader.getFieldAs<double>(dim, in_idx);
            las_distances_writer.setField(dim, out_idx, value);
        }
    }

    std::cout << "Computing distances..." << std::endl;
    auto indices_groups = points.get_groups_in_gps_time_order();

    // Create a progress bar
    ProgressBarTotal bar(
        indices_groups.size(), "Computing distances",
        indicators::option::ForegroundColor{indicators::Color::green});
    bar.start();

    std::size_t multi_echo_missing_return_count = 0;
    for (const auto &group : indices_groups) {
        if (group.empty()) {
            throw std::runtime_error("Empty group of points.");
        }

        if (group.size() > 1) {
            // Multi-echo
            // std::cout << "Multi-echo group with " << group.size() << "
            // points."
            //           << std::endl;

            const auto number_of_returns = group.size();
            std::optional<std::size_t> lowest_return_idx, highest_return_idx;
            for (auto idx : group) {
                auto return_number = points[idx].return_number;
                if (return_number == number_of_returns) {
                    lowest_return_idx = idx;
                } else if (return_number == 1) {
                    highest_return_idx = idx;
                }
            }

            if (!highest_return_idx || !lowest_return_idx) {
                multi_echo_missing_return_count++;
                continue;
            }

            for (auto idx : group) {
                double down_signed_vert_gap, up_signed_vert_gap;

                auto p0 = points[idx];
                auto ph = points[*highest_return_idx];
                auto pl = points[*lowest_return_idx];

                down_signed_vert_gap = p0.signed_vertical_distance_to(pl);
                up_signed_vert_gap = p0.signed_vertical_distance_to(ph);

                bool is_roof_edge = down_signed_vert_gap < MAX_VERT_GAP_FOOT;
                bool is_foot_edge = up_signed_vert_gap > MIN_VERT_GAP_ROOF;

                std::size_t out_idx = points.to_sorted_index(idx);
                las_distances_writer.setField(
                    CustomDimensions::Id::DownSignedVertGap, out_idx,
                    down_signed_vert_gap);
                las_distances_writer.setField(
                    CustomDimensions::Id::UpSignedVertGap, out_idx,
                    up_signed_vert_gap);
                las_distances_writer.setField(CustomDimensions::Id::IsRoofEdge,
                                              out_idx, is_roof_edge);
                las_distances_writer.setField(CustomDimensions::Id::IsFootEdge,
                                              out_idx, is_foot_edge);

                // Add the points that are roof edges
                if (is_roof_edge) {
                    std::size_t edge_idx = las_edge_writer.pointCount();
                    las_edge_writer.setField(pdal::Dimension::Id::X, edge_idx,
                                             p0.x);
                    las_edge_writer.setField(pdal::Dimension::Id::Y, edge_idx,
                                             p0.y);
                    las_edge_writer.setField(pdal::Dimension::Id::Z, edge_idx,
                                             p0.z);
                    las_edge_writer.setField(
                        pdal::Dimension::Id::Classification, edge_idx,
                        static_cast<
                            std::underlying_type_t<LASclassification::Value>>(
                            p0.classification));
                    las_edge_writer.setField(CustomDimensions::Id::IsGenerated,
                                             edge_idx, false);
                }
            }

        } else {
            // Single echo
            // std::cout << "Single-echo group" << std::endl;

            const auto idx = group[0];
            const auto p = points[idx];
            const auto gps_time = points[idx].gps_time;
            const auto prev_gps_time = points.get_prev_gps_time(gps_time);
            const auto next_gps_time = points.get_next_gps_time(gps_time);
            const auto prev_group =
                points.get_indices_for_prev_gps_time(gps_time);
            const auto next_group =
                points.get_indices_for_next_gps_time(gps_time);

            // Only consider the last return of a multi-echo neighbour
            double down_signed_vert_gap = 0.0;
            double up_signed_vert_gap = 0.0;
            std::optional<Point3D> p_down;
            std::optional<Point3D> p_other;

            if (!prev_group.empty() && prev_gps_time.has_value()) {
                auto prev_idx_lowest_return =
                    points.get_index_of_lowest_return(prev_gps_time.value());
                Point3D p1 = points[prev_idx_lowest_return];
                double signed_vert_gap = p.signed_vertical_distance_to(p1);
                if (signed_vert_gap < down_signed_vert_gap) {
                    down_signed_vert_gap = signed_vert_gap;
                    p_down = p1;
                    // Find the closest point among the other group
                    double distance_p_down =
                        std::numeric_limits<double>::infinity();
                    for (auto idx : next_group) {
                        Point3D p2 = points[idx];
                        double distance = p2.distance_to(*p_down);
                        if (distance < distance_p_down) {
                            distance_p_down = distance;
                            p_other = p2;
                        }
                    }
                }
                if (signed_vert_gap > up_signed_vert_gap) {
                    up_signed_vert_gap = signed_vert_gap;
                }
            }
            if (!next_group.empty() && next_gps_time.has_value()) {
                Point3D p1 = points[points.get_index_of_lowest_return(
                    next_gps_time.value())];
                double signed_vert_gap = p.signed_vertical_distance_to(p1);
                if (signed_vert_gap < down_signed_vert_gap) {
                    down_signed_vert_gap = signed_vert_gap;
                    p_down = p1;
                    // Find the closest point among the other group
                    double distance_p_down =
                        std::numeric_limits<double>::infinity();
                    for (auto idx : prev_group) {
                        Point3D p2 = points[idx];
                        double distance = p2.distance_to(*p_down);
                        if (distance < distance_p_down) {
                            distance_p_down = distance;
                            p_other = p2;
                        }
                    }
                }
                if (signed_vert_gap > up_signed_vert_gap) {
                    up_signed_vert_gap = signed_vert_gap;
                }
            }

            bool is_roof_edge = down_signed_vert_gap < MAX_VERT_GAP_FOOT;
            bool is_foot_edge = up_signed_vert_gap > MIN_VERT_GAP_ROOF;

            std::size_t out_idx = points.to_sorted_index(idx);
            las_distances_writer.setField(
                CustomDimensions::Id::DownSignedVertGap, out_idx,
                down_signed_vert_gap);
            las_distances_writer.setField(CustomDimensions::Id::UpSignedVertGap,
                                          out_idx, up_signed_vert_gap);
            las_distances_writer.setField(CustomDimensions::Id::IsRoofEdge,
                                          out_idx, is_roof_edge);
            las_distances_writer.setField(CustomDimensions::Id::IsFootEdge,
                                          out_idx, is_foot_edge);

            // Add the points that are roof edges
            if (is_roof_edge && p_down && p_other) {
                std::size_t edge_idx = las_edge_writer.pointCount();
                Point3D edge_point;
                if (p_other->is_neighbour_in_distance(p)) {
                    edge_point = p + (p - *p_other) / 2;
                } else {
                    Point3D p_average = (*p_down + p) / 2;
                    edge_point = Point3D(p_average.x, p_average.y, p.z);
                }
                las_edge_writer.setField(pdal::Dimension::Id::X, edge_idx,
                                         edge_point.x);
                las_edge_writer.setField(pdal::Dimension::Id::Y, edge_idx,
                                         edge_point.y);
                las_edge_writer.setField(pdal::Dimension::Id::Z, edge_idx,
                                         edge_point.z);
                las_edge_writer.setField(
                    pdal::Dimension::Id::Classification, edge_idx,
                    static_cast<
                        std::underlying_type_t<LASclassification::Value>>(
                        p.classification));
                las_edge_writer.setField(CustomDimensions::Id::IsGenerated,
                                         edge_idx, true);
            }
        }
        bar.increment(1);
    }
    bar.finish();

    std::cout << "Number of multi-echo rays missing first or last return: "
              << multi_echo_missing_return_count << std::endl;

    // Filter out points with classification not in the allowed classes
    const std::vector<LASclassification::Value> allowed_classes = {
        // LASclassification::Unclassified,
        // LASclassification::Unassigned,
        // LASclassification::Ground,
        // LASclassification::Building,
        // LASclassification::PermanentOverground,
        // LASclassification::MiscellaneousBuildings,
    };

    std::cout << "Writing output LAS file..." << std::endl;
    las_distances_writer.write(output_distances_file, allowed_classes);
    las_edge_writer.write(output_edges_file, allowed_classes);

    std::cout << "Done." << std::endl;
}
