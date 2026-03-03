#include "distances.hpp"

#include <CGAL/Kernel/global_functions_3.h>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <sys/types.h>
#include <type_traits>

#include <pdal/Dimension.hpp>
#include <pdal/pdal_types.hpp>

#include "las/reader.hpp"
#include "las/trajectory.hpp"
#include "las/writer.hpp"
#include "pbar.hpp"
#include "points.hpp"
#include "utils/cgal.hpp"

const double MIN_VERT_GAP_ROOF = 1.0;
const double MAX_VERT_GAP_FOOT = -1.0;
const double MIN_VERT_GAIN_ROOF = 2.0;

void _old_compute_distances_in_order(const std::string &input_points_file,
                                     const std::string &input_trajectory_file,
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
    CustomLasReader las_reader(input_points_file);
    las_reader.execute();
    auto in_view = las_reader.point_view();
    auto [predefined_dims, proprietary_dims] = las_reader.dimensions();
    auto n_features = las_reader.point_count();

    std::cout << "Number of points: " << n_features << std::endl;

    // Prepare the points with attributes
    Points3DAttrGPSSorted points(in_view);

    // Prepare the trajectory
    std::cout << "Reading trajectory file..." << std::endl;
    Trajectory trajectory = read_trajectory(input_trajectory_file);

    // Prepare the output writers
    std::cout << "Preparing output objects..." << std::endl;
    std::vector<pdal::Dimension::Id> distances_dims = predefined_dims;
    std::vector<ProprietaryDimension> distances_custom_dims = proprietary_dims;
    std::vector<ProprietaryDimension> new_distances_custom_dims = {
        CustomDimensions::Id::ReturnNumberComputed,
        CustomDimensions::Id::NumberOfReturnsComputed,
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

        las_distances_writer.setField(
            CustomDimensions::Id::ReturnNumberComputed, out_idx,
            points[in_idx].get_return_number_computed());
        las_distances_writer.setField(
            CustomDimensions::Id::NumberOfReturnsComputed, out_idx,
            points[in_idx].get_number_of_returns_computed());
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
                auto return_number = points[idx].get_return_number_computed();
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
                                             edge_idx, 0);
                }
            }

        } else {
            // Single echo
            // std::cout << "Single-echo group" << std::endl;

            const auto idx = group[0];
            const auto p = points[idx];
            const auto gps_time = points[idx].gps_time;
            const auto prev_gps_time_ = points.get_prev_gps_time(gps_time);
            const auto next_gps_time_ = points.get_next_gps_time(gps_time);
            const auto prev_group =
                points.get_indices_for_prev_gps_time(gps_time);
            const auto next_group =
                points.get_indices_for_next_gps_time(gps_time);

            if (!prev_gps_time_ or !next_gps_time_) {
                continue;
            }
            const auto prev_gps_time = *prev_gps_time_;
            const auto next_gps_time = *next_gps_time_;

            const auto prev_prev_gps_time_ =
                points.get_prev_gps_time(prev_gps_time);
            const auto next_next_gps_time_ =
                points.get_next_gps_time(next_gps_time);

            // Only consider the last return of a multi-echo neighbour
            double down_signed_vert_gap = 0.0;
            double up_signed_vert_gap = 0.0;
            std::optional<Point3D> p_down;
            std::optional<Point3D> p_other;
            std::optional<Point3D> p_other_2;

            if (!prev_group.empty() && next_next_gps_time_.has_value()) {
                auto next_next_gps_time = *next_next_gps_time_;
                auto prev_idx_lowest_return =
                    points.get_index_of_lowest_return(prev_gps_time);
                Point3D p1 = points[prev_idx_lowest_return];
                double signed_vert_gap = p.signed_vertical_distance_to(p1);
                if (signed_vert_gap < down_signed_vert_gap) {
                    down_signed_vert_gap = signed_vert_gap;
                    p_down = p1;
                    // Find the closest point among the next group
                    double distance_p_down =
                        std::numeric_limits<double>::infinity();
                    for (auto idx : next_group) {
                        Point3D p2 = points[idx];
                        double distance_p2 = p2.distance_to(*p_down);
                        if (distance_p2 < distance_p_down) {
                            distance_p_down = distance_p2;
                            p_other = p2;

                            // Find the point among the next next group that is
                            // closest to p_other
                            double distance_p_other =
                                std::numeric_limits<double>::infinity();
                            for (auto idx : points.get_indices_for_gps_time(
                                     next_next_gps_time)) {
                                Point3D p3 = points[idx];
                                double distance_p3 = p3.distance_to(*p_other);
                                if (distance_p3 < distance_p_other) {
                                    distance_p_other = distance_p3;
                                    p_other_2 = p3;
                                }
                            }
                        }
                    }
                }
                if (signed_vert_gap > up_signed_vert_gap) {
                    up_signed_vert_gap = signed_vert_gap;
                }
            }
            if (!next_group.empty() && prev_prev_gps_time_.has_value()) {
                auto prev_prev_gps_time = *prev_prev_gps_time_;
                auto next_idx_lowest_return =
                    points.get_index_of_lowest_return(next_gps_time);
                Point3D p1 = points[next_idx_lowest_return];
                double signed_vert_gap = p.signed_vertical_distance_to(p1);
                if (signed_vert_gap < down_signed_vert_gap) {
                    down_signed_vert_gap = signed_vert_gap;
                    p_down = p1;
                    // Find the closest point among the previous group
                    double distance_p_down =
                        std::numeric_limits<double>::infinity();
                    for (auto idx : prev_group) {
                        Point3D p2 = points[idx];
                        double distance_p2 = p2.distance_to(*p_down);
                        if (distance_p2 < distance_p_down) {
                            distance_p_down = distance_p2;
                            p_other = p2;

                            // Find the point among the previous previous group
                            // that is closest to p_other
                            double distance_p_other =
                                std::numeric_limits<double>::infinity();
                            for (auto idx : points.get_indices_for_gps_time(
                                     prev_prev_gps_time)) {
                                Point3D p3 = points[idx];
                                double distance_p3 = p3.distance_to(*p_other);
                                if (distance_p3 < distance_p_other) {
                                    distance_p_other = distance_p3;
                                    p_other_2 = p3;
                                }
                            }
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
            if (is_roof_edge && p_down && p_other && p_other_2) {
                std::size_t edge_idx = las_edge_writer.pointCount();
                Point3D edge_point;
                uint8_t is_generated;

                Point_3 p_cgal(p.x, p.y, p.z);
                Point_3 p_down_cgal(p_down->x, p_down->y, p_down->z);
                Point_3 p_other_cgal(p_other->x, p_other->y, p_other->z);
                Point_3 p_other_2_cgal(p_other_2->x, p_other_2->y,
                                       p_other_2->z);

                if (CustomCGAL::are_almost_collinear(
                        p_cgal, p_other_cgal, p_other_2_cgal,
                        CustomCGAL::Angle::from_degrees(5))) {
                    edge_point = p + (p - *p_other) / 2;
                    is_generated = 1;
                } else {
                    Point_3 scanner_position_cgal =
                        trajectory.get_point_at_gps_time(gps_time);
                    Vector_3 scanner_to_p = p_cgal - scanner_position_cgal;
                    Vector_3 scanner_to_down =
                        p_down_cgal - scanner_position_cgal;
                    Vector_3 scanner_to_edge =
                        (scanner_to_p + scanner_to_down) / 2.0;
                    scanner_to_edge =
                        scanner_to_edge /
                        std::sqrt(scanner_to_edge.squared_length()) *
                        std::sqrt(scanner_to_p.squared_length());

                    // Point3D p_average = (*p_down + p) / 2;
                    // edge_point = Point3D(p_average.x, p_average.y, p.z);
                    Point_3 edge_point_cgal =
                        scanner_position_cgal + scanner_to_edge;
                    edge_point =
                        Point3D(edge_point_cgal.x(), edge_point_cgal.y(),
                                edge_point_cgal.z());
                    is_generated = 2;
                }

                // if (p_other->is_neighbour_in_distance(p)) {
                //     edge_point = p + (p - *p_other) / 2;
                // } else {
                //     Point3D p_average = (*p_down + p) / 2;
                //     edge_point = Point3D(p_average.x, p_average.y, p.z);
                // }
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
                                         edge_idx, is_generated);
            }
        }
        bar.increment(1);
    }
    bar.finish();

    // std::cout << "Number of multi-echo rays missing first or last return: "
    //           << multi_echo_missing_return_count << std::endl;

    // Filter out points with classification not in the allowed classes
    const std::vector<LASclassification::Value> allowed_classes = {
        LASclassification::Value::Unclassified,
        LASclassification::Value::Unassigned,
        LASclassification::Value::Ground,
        LASclassification::Value::Building,
        LASclassification::Value::PermanentOverground,
        LASclassification::Value::MiscellaneousBuildings,
    };

    std::cout << "Writing output LAS file..." << std::endl;
    las_distances_writer.write(output_distances_file, allowed_classes);
    las_edge_writer.write(output_edges_file, allowed_classes);

    std::cout << "Done." << std::endl;
}

uint8_t compute_roof_likelihood(Point_3 p, std::optional<Point_3> p_roof,
                                std::optional<Point_3> p_roof_2) {
    if (!p_roof || !p_roof_2) {
        return 0;
    }

    return CustomCGAL::are_almost_collinear(p, *p_roof, *p_roof_2,
                                            CustomCGAL::Angle::from_degrees(5));
}

std::optional<PtsStructs::PointId>
find_closest_point(PtsStructs::PointId point_id,
                   std::optional<PtsStructs::RayId> neighbour_ray_id,
                   PtsStructs::Topology3D &topo) {
    if (!neighbour_ray_id) {
        return std::nullopt;
    }

    PtsStructs::Ray3D neighbour_ray = topo.get_ray(*neighbour_ray_id);
    if (neighbour_ray.empty()) {
        return std::nullopt;
    }

    Point_3 p = topo.points->get_point(point_id);
    std::optional<PtsStructs::PointId> closest_point_id;
    double closest_distance = std::numeric_limits<double>::infinity();
    for (PtsStructs::PointId neighbour_point_id :
         neighbour_ray.get_point_ids()) {
        Point_3 neighbour_p = topo.points->get_point(neighbour_point_id);
        double distance = CGAL::squared_distance(p, neighbour_p);
        if (distance < closest_distance) {
            closest_distance = distance;
            closest_point_id = neighbour_point_id;
        }
    }

    return closest_point_id;
}

// Function that computes the roof likelihood for a point given its point ID and
// the function that gives the neighbouring ray ID for a given ray ID
uint8_t compute_roof_likelihood(
    PtsStructs::PointId point_id,
    std::function<std::optional<PtsStructs::RayId>(PtsStructs::RayId)>
        get_neighbour_ray_id,
    PtsStructs::Topology3D &topo) {

    // Find the two points in the neighbouring rays that are closest to the
    // point with the given point ID
    auto ray_id = topo.get_ray_id_from_point_id(point_id);
    auto neighbour_ray_id = get_neighbour_ray_id(ray_id);
    auto neighbour_point_id =
        find_closest_point(point_id, neighbour_ray_id, topo);
    if (!neighbour_point_id) {
        return 0;
    }
    auto neighbour_2_ray_id = get_neighbour_ray_id(*neighbour_ray_id);
    auto neighbour_2_point_id =
        find_closest_point(*neighbour_point_id, neighbour_2_ray_id, topo);
    if (!neighbour_2_point_id) {
        return 0;
    }

    // Compute the roof likelihood
    Point_3 p = topo.points->get_point(point_id);
    Point_3 p_neighbour = topo.points->get_point(*neighbour_point_id);
    Point_3 p_neighbour_2 = topo.points->get_point(*neighbour_2_point_id);

    return compute_roof_likelihood(p, p_neighbour, p_neighbour_2);
}

const double MAX_DISTANCE_NEIGHBOURS_ON_ROOF = 1.0;

void compute_distances_in_order(const std::string &input_points_file,
                                const std::string &input_trajectory_file,
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
    NewLasReader las_reader(input_points_file);
    std::cout << "Number of points: " << las_reader.points->point_count()
              << std::endl;
    auto [predefined_dims, proprietary_dims] = las_reader.points->dimensions();
    auto n_features = las_reader.points->point_count();

    std::cout << "Number of points: " << n_features << std::endl;

    // Prepare the trajectory
    std::cout << "Reading trajectory file..." << std::endl;
    Trajectory trajectory = read_trajectory(input_trajectory_file);

    // Prepare the points with attributes
    PtsStructs::Topology3D topo(las_reader.points, trajectory);

    // Prepare the output writers
    std::cout << "Preparing output objects..." << std::endl;
    std::vector<pdal::Dimension::Id> distances_dims = predefined_dims;
    std::vector<ProprietaryDimension> distances_custom_dims = proprietary_dims;
    std::vector<ProprietaryDimension> new_distances_custom_dims = {
        CustomDimensions::Id::ReturnNumberComputed,
        CustomDimensions::Id::NumberOfReturnsComputed,
        CustomDimensions::Id::VerticalGain,
        CustomDimensions::Id::IsRoofEdge,
        CustomDimensions::Id::IsFootEdge,
    };
    distances_custom_dims.insert(distances_custom_dims.end(),
                                 new_distances_custom_dims.begin(),
                                 new_distances_custom_dims.end());
    NewLasWriter las_distances_writer(distances_dims, distances_custom_dims,
                                      las_reader.points->spatial_reference());

    std::vector<pdal::Dimension::Id> edge_dims = {
        pdal::Dimension::Id::X,
        pdal::Dimension::Id::Y,
        pdal::Dimension::Id::Z,
        pdal::Dimension::Id::Classification,
    };
    std::vector<ProprietaryDimension> edge_custom_dims = {
        CustomDimensions::Id::IsGenerated,
        CustomDimensions::Id::VerticalGain,
        CustomDimensions::Id::RoofLikelihood,
    };
    NewLasWriter las_edge_writer(edge_dims, edge_custom_dims,
                                 las_reader.points->spatial_reference());

    std::cout << "Adding existing dimensions to output view..." << std::endl;
    // Add the existing dimensions to the output point view
    // The points need to be processed in the order of the output view
    for (PtsStructs::PointId idx = 0; idx < n_features; ++idx) {
        for (auto dim : predefined_dims) {
            double value = las_reader.points->get_field_as<double>(dim, idx);
            las_distances_writer.points->set_field(dim, idx, value);
            // Get the value of the dimension in the correct type and set it in
            // the output view
            // char *value;
            // in_view->getField(value, dim, pdal::Dimension::defaultType(dim),
            //                   in_idx);
            // las_distances_writer.setField(dim, out_idx, value);
        }
        for (auto dim : proprietary_dims) {
            auto value = las_reader.points->get_field_as<double>(dim, idx);
            las_distances_writer.points->set_field(dim, idx, value);
        }

        PtsStructs::RayId ray_id = topo.get_ray_id_from_point_id(idx);
        PtsStructs::Ray3D ray = topo.get_ray(ray_id);
        auto return_number_computed = ray.get_return_number(idx);
        auto number_of_returns_computed = ray.get_number_of_returns();

        las_distances_writer.points->set_field(
            CustomDimensions::Id::ReturnNumberComputed, idx,
            return_number_computed);
        las_distances_writer.points->set_field(
            CustomDimensions::Id::NumberOfReturnsComputed, idx,
            number_of_returns_computed);
    }

    std::cout << "Computing distances..." << std::endl;

    // Create a progress bar
    auto ray_count = topo.ray_count();
    ProgressBarTotal bar(
        ray_count, "Computing distances",
        indicators::option::ForegroundColor{indicators::Color::green});
    bar.start();

    std::size_t count_single_echo = 0;
    std::size_t count_multi_echo = 0;

    for (PtsStructs::RayId ray_id = 0; ray_id < ray_count; ++ray_id) {
        PtsStructs::Ray3D ray = topo.get_ray(ray_id);

        if (ray.empty()) {
            throw std::runtime_error("Empty group of points.");
        }

        if (ray.size() > 1) {
            // Multi-echo

            PtsStructs::PointId highest_return_idx =
                ray.get_point_id_in_return_order(-1);
            auto p_high = topo.points->get_point(highest_return_idx);

            for (PtsStructs::PointId idx : ray.get_point_ids()) {
                double vertical_gain;

                auto p0 = topo.points->get_point(idx);

                vertical_gain = p0.z() - p_high.z();
                bool is_roof_edge = vertical_gain > MIN_VERT_GAIN_ROOF;

                las_distances_writer.points->set_field(
                    CustomDimensions::Id::VerticalGain, idx, vertical_gain);
                las_distances_writer.points->set_field(
                    CustomDimensions::Id::IsRoofEdge, idx, is_roof_edge);

                // Add the points that are roof edges
                if (is_roof_edge) {
                    uint8_t is_generated = 0;

                    uint8_t roof_likelihood = compute_roof_likelihood(
                        idx,
                        [&](PtsStructs::RayId r_id) {
                            return topo.get_next_ray_in_scan_line(r_id);
                        },
                        topo);

                    std::size_t edge_idx =
                        las_edge_writer.points->point_count();
                    las_edge_writer.points->set_point(edge_idx, p0);
                    las_edge_writer.points->copy_field<int>(
                        pdal::Dimension::Id::Classification, edge_idx,
                        topo.points, idx);
                    las_edge_writer.points->set_field(
                        CustomDimensions::Id::IsGenerated, edge_idx,
                        is_generated);
                    las_edge_writer.points->set_field(
                        CustomDimensions::Id::VerticalGain, edge_idx,
                        vertical_gain);
                    // TODO: compute roof likelihood for multi-echo points
                    las_edge_writer.points->set_field(
                        CustomDimensions::Id::RoofLikelihood, edge_idx,
                        roof_likelihood);

                    count_multi_echo++;
                }
            }

        } else {
            // Single echo

            // Create all the tuples of neighbours to consider
            std::vector<
                std::tuple<PtsStructs::RayId, std::optional<PtsStructs::RayId>,
                           std::optional<PtsStructs::RayId>>>
                neighbour_ray_ids;
            const auto prev_ray_id = topo.get_prev_ray_in_scan_line(ray_id);
            const auto next_ray_id = topo.get_next_ray_in_scan_line(ray_id);
            const auto prev_prev_ray_id =
                prev_ray_id.has_value()
                    ? topo.get_prev_ray_in_scan_line(*prev_ray_id)
                    : std::nullopt;
            const auto next_next_ray_id =
                next_ray_id.has_value()
                    ? topo.get_next_ray_in_scan_line(*next_ray_id)
                    : std::nullopt;

            if (prev_ray_id.has_value()) {
                neighbour_ray_ids.emplace_back(*prev_ray_id, next_ray_id,
                                               next_next_ray_id);
            }
            if (next_ray_id.has_value()) {
                neighbour_ray_ids.emplace_back(*next_ray_id, prev_ray_id,
                                               prev_prev_ray_id);
            }

            PtsStructs::PointId p_id = ray.get_point_id_in_return_order(-1);
            Point_3 p = topo.points->get_point(p_id);

            double vertical_gain = 0.0;
            std::optional<Point_3> p_ground;
            std::optional<Point_3> p_roof;
            std::optional<Point_3> p_roof_2;

            for (const auto &[neighbour_ray_id, opposite_neighbour_ray_id,
                              opposite_neighbour_ray_id_2] :
                 neighbour_ray_ids) {
                PtsStructs::Ray3D neighbour_ray =
                    topo.get_ray(neighbour_ray_id);
                if (neighbour_ray.empty()) {
                    continue;
                }
                // Only consider the last return of a multi-echo neighbour
                PtsStructs::PointId neighbour_highest_return_idx =
                    neighbour_ray.get_point_id_in_return_order(-1);
                Point_3 p_neighbour_highest_return =
                    topo.points->get_point(neighbour_highest_return_idx);
                double vertical_gain_ = p.z() - p_neighbour_highest_return.z();
                if (vertical_gain_ > vertical_gain) {
                    vertical_gain = vertical_gain_;
                    p_ground = p_neighbour_highest_return;

                    // Find the closest point among the opposite neighbours
                    if (!opposite_neighbour_ray_id.has_value()) {
                        continue;
                    }
                    auto p_roof_id = find_closest_point(
                        p_id, opposite_neighbour_ray_id, topo);
                    if (!p_roof_id.has_value()) {
                        continue;
                    }
                    p_roof = topo.points->get_point(*p_roof_id);

                    // Same for the neighbour of the opposite neighbour
                    if (!opposite_neighbour_ray_id_2.has_value()) {
                        continue;
                    }
                    auto p_roof_2_id = find_closest_point(
                        *p_roof_id, opposite_neighbour_ray_id_2, topo);
                    if (!p_roof_2_id.has_value()) {
                        continue;
                    }
                    p_roof_2 = topo.points->get_point(*p_roof_2_id);
                }
            }

            bool is_roof_edge = vertical_gain > MIN_VERT_GAIN_ROOF;

            las_distances_writer.points->set_field(
                CustomDimensions::Id::VerticalGain, p_id, vertical_gain);
            las_distances_writer.points->set_field(
                CustomDimensions::Id::IsRoofEdge, p_id, is_roof_edge);

            // Add the points that are roof edges
            if (is_roof_edge) {
                double gps_time = ray.get_gps_time();
                Point_3 edge_point;
                uint8_t is_generated;
                uint8_t roof_likelihood;

                // Determine whether to compute the edge point by extending the
                // roof or by averaging the scanner position and the ground
                // point
                // TODO Look into this
                bool extend_roof = false;
                if (p_roof && p_roof_2 &&
                    CustomCGAL::are_almost_collinear(
                        p, *p_roof, *p_roof_2,
                        CustomCGAL::Angle::from_degrees(5))) {
                    double distance_p_roof =
                        std::sqrt(CGAL::squared_distance(p, *p_roof));
                    double distance_p_roof_2 =
                        std::sqrt(CGAL::squared_distance(p, *p_roof_2));
                    if (distance_p_roof < MAX_DISTANCE_NEIGHBOURS_ON_ROOF &&
                        distance_p_roof_2 < MAX_DISTANCE_NEIGHBOURS_ON_ROOF) {
                        extend_roof = true;
                    }
                }

                if (extend_roof) {
                    edge_point = p + (p - *p_roof) / 2;
                    is_generated = 1;
                    roof_likelihood = 1;
                } else if (p_ground) {
                    Point_3 scanner_position =
                        trajectory.get_point_at_gps_time(gps_time);
                    Vector_3 scanner_to_p = p - scanner_position;
                    Vector_3 scanner_to_down = *p_ground - scanner_position;
                    Vector_3 scanner_to_edge =
                        (scanner_to_p + scanner_to_down) / 2.0;
                    scanner_to_edge =
                        scanner_to_edge /
                        std::sqrt(scanner_to_edge.squared_length()) *
                        std::sqrt(scanner_to_p.squared_length());

                    // Point3D p_average = (*p_down + p) / 2;
                    // edge_point = Point3D(p_average.x, p_average.y, p.z);
                    edge_point = scanner_position + scanner_to_edge;
                    is_generated = 1;
                    roof_likelihood = 0;
                } else {
                    throw(std::runtime_error("Cannot compute edge point for "
                                             "single-echo ray with gps time " +
                                             std::to_string(gps_time)));
                }

                std::size_t edge_idx = las_edge_writer.points->point_count();
                las_edge_writer.points->set_point(edge_idx, edge_point);
                las_edge_writer.points->copy_field<
                    std::underlying_type_t<LASclassification::Value>>(
                    pdal::Dimension::Id::Classification, edge_idx, topo.points,
                    p_id);
                las_edge_writer.points->set_field(
                    CustomDimensions::Id::IsGenerated, edge_idx, is_generated);
                las_edge_writer.points->set_field(
                    CustomDimensions::Id::VerticalGain, edge_idx,
                    vertical_gain);
                las_edge_writer.points->set_field(
                    CustomDimensions::Id::RoofLikelihood, edge_idx,
                    roof_likelihood);

                count_single_echo++;
            }
        }
        bar.increment(1);
    }
    bar.finish();

    std::cout << "Number of single-echo rays giving a roof edge: "
              << count_single_echo << std::endl;
    std::cout << "Number of multi-echo rays giving a roof edge: "
              << count_multi_echo << std::endl;

    // std::cout << "Number of multi-echo rays missing first or last return: "
    //           << multi_echo_missing_return_count << std::endl;

    // Filter out points with classification not in the allowed classes
    const std::vector<LASclassification::Value> allowed_classes = {
        LASclassification::Value::Unclassified,
        LASclassification::Value::Unassigned,
        LASclassification::Value::Ground,
        LASclassification::Value::Building,
        LASclassification::Value::PermanentOverground,
        LASclassification::Value::MiscellaneousBuildings,
    };

    std::cout << "Writing output LAS file..." << std::endl;
    las_distances_writer.write(output_distances_file, allowed_classes);
    las_edge_writer.write(output_edges_file, allowed_classes);

    std::cout << "Done." << std::endl;
}
