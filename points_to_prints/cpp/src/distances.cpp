#include "distances.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <sys/types.h>
#include <type_traits>
#include <vector>

#include <CGAL/Kernel/global_functions_3.h>
#include <pdal/Dimension.hpp>
#include <pdal/pdal_types.hpp>

#include "las/reader.hpp"
#include "las/trajectory.hpp"
#include "las/writer.hpp"
#include "pbar.hpp"
#include "pca.hpp"
#include "points.hpp"
#include "utils/cgal.hpp"

const double MIN_VERT_GAIN_ROOF = 1.0;
const double MAX_DISTANCE_NEIGHBOURS_ON_ROOF = 1.0;

enum class Direction {
    Previous,
    Next,
};

std::optional<PtsStructs::PointId>
find_closest_point(std::optional<PtsStructs::PointId> point_id,
                   std::optional<PtsStructs::RayId> neighbour_ray_id,
                   PtsStructs::Topology3D &topo) {
    if (!point_id) {
        return std::nullopt;
    }
    if (!neighbour_ray_id) {
        return std::nullopt;
    }

    const auto &neighbour_ray = topo.get_ray(*neighbour_ray_id);
    if (neighbour_ray.empty()) {
        return std::nullopt;
    }

    Point_3 p = topo.points->get_point(*point_id);
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

std::tuple<double, double> compute_planarity_and_horizontality(
    PtsStructs::PointId point_id,
    std::vector<std::optional<PtsStructs::RayId>> neighbour_ray_ids,
    PtsStructs::Topology3D &topo) {

    // Find the closest point among the neighbours in the neighbouring rays
    std::vector<PtsStructs::PointId> neighbour_point_ids;
    std::size_t neighbour_ground_count = 0;
    for (auto neighbour_ray_id : neighbour_ray_ids) {
        auto neighbour_point_id =
            find_closest_point(point_id, neighbour_ray_id, topo);
        if (neighbour_point_id) {
            neighbour_point_ids.push_back(*neighbour_point_id);

            const auto cls_raw = topo.points->get_field_as<
                std::underlying_type_t<LASclassification::Value>>(
                pdal::Dimension::Id::Classification, *neighbour_point_id);
            const auto cls = static_cast<LASclassification::Value>(cls_raw);
            if (cls == LASclassification::Value::Ground) {
                neighbour_ground_count++;
            }
        }
    }

    // Check that there are enough neighbours to estimate a plane
    if (neighbour_point_ids.size() < 2) {
        return std::make_tuple(0.0, 0.0);
    }

    // Check that no neighbour is classified as ground, to avoid biasing the
    // normal vector
    if (neighbour_ground_count > 0) {
        return std::make_tuple(0.0, 0.0);
    }

    // Compute PCA to get eigenvalues and normals
    std::vector<Point_3> points;
    // points.push_back(topo.points->get_point(point_id));
    for (auto neighbour_point_id : neighbour_point_ids) {
        points.push_back(topo.points->get_point(neighbour_point_id));
    }
    auto [normal_vector, tangent_plane, eigenvalues] = compute_pca_once(points);

    // Compute the planarity
    double planarity =
        (eigenvalues.middle - eigenvalues.smallest) / eigenvalues.largest;

    // Compute the horizontality
    CustomCGAL::Angle angle =
        CustomCGAL::angle(normal_vector, Vector_3(0, 0, 1));
    if (angle.in_degrees() > 180) {
        angle = CustomCGAL::Angle::from_degrees(360 - angle.in_degrees());
    }
    if (angle.in_degrees() > 90) {
        angle = CustomCGAL::Angle::from_degrees(180 - angle.in_degrees());
    }
    double horizontality = 1 - angle.in_degrees() / 90.0;

    return std::make_tuple(planarity, horizontality);
}

std::optional<std::tuple<double, double>> compute_planarity_and_horizontality(
    PtsStructs::PointId point_id,
    std::optional<PtsStructs::ScanLineId> scan_line_1_id,
    std::optional<PtsStructs::ScanLineId> scan_line_2_id, Direction direction,
    PtsStructs::Topology3D &topo) {
    if (!scan_line_1_id || !scan_line_2_id) {
        return std::nullopt;
    }
    const auto &scan_line_0 =
        topo.get_scan_line(topo.get_scan_line_id(topo.get_ray_id(point_id)));
    const auto &scan_line_1 = topo.get_scan_line(*scan_line_1_id);
    const auto &scan_line_2 = topo.get_scan_line(*scan_line_2_id);
    PtsStructs::RayId ray_0_0_id = topo.get_ray_id(point_id);
    std::function<std::optional<PtsStructs::RayId>(
        std::optional<PtsStructs::RayId>)>
        iter_func_0, iter_func_1, iter_func_2;
    if (direction == Direction::Previous) {
        iter_func_0 = [&scan_line_0](std::optional<PtsStructs::RayId> ray_id) {
            return scan_line_0.get_prev_ray_id(ray_id);
        };
        iter_func_1 = [&scan_line_1](std::optional<PtsStructs::RayId> ray_id) {
            return scan_line_1.get_prev_ray_id(ray_id);
        };
        iter_func_2 = [&scan_line_2](std::optional<PtsStructs::RayId> ray_id) {
            return scan_line_2.get_prev_ray_id(ray_id);
        };
    } else {
        iter_func_0 = [&scan_line_0](std::optional<PtsStructs::RayId> ray_id) {
            return scan_line_0.get_next_ray_id(ray_id);
        };
        iter_func_1 = [&scan_line_1](std::optional<PtsStructs::RayId> ray_id) {
            return scan_line_1.get_next_ray_id(ray_id);
        };
        iter_func_2 = [&scan_line_2](std::optional<PtsStructs::RayId> ray_id) {
            return scan_line_2.get_next_ray_id(ray_id);
        };
    }
    const auto ray_0_1_id = iter_func_0(ray_0_0_id);
    const auto ray_0_2_id = iter_func_0(ray_0_1_id);

    if (!ray_0_1_id) {
        return std::make_tuple<double, double>(0.0, 0.0);
    }

    const auto ray_1_1_id =
        scan_line_1.get_closest_ray_by_two_directions(ray_0_0_id, *ray_0_1_id);
    const auto ray_1_2_id = iter_func_1(ray_1_1_id);
    const auto ray_2_1_id =
        scan_line_2.get_closest_ray_by_two_directions(ray_0_0_id, *ray_0_1_id);
    const auto ray_2_2_id = iter_func_2(ray_2_1_id);

    std::vector<std::optional<PtsStructs::RayId>> neighbour_ray_ids = {
        ray_0_1_id, ray_0_2_id, ray_1_1_id, ray_1_2_id, ray_2_1_id, ray_2_2_id};
    return compute_planarity_and_horizontality(point_id, neighbour_ray_ids,
                                               topo);
}

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
        CustomDimensions::Id::Planarity,
        CustomDimensions::Id::Horizontality,
    };
    NewLasWriter las_edge_writer(edge_dims, edge_custom_dims,
                                 las_reader.points->spatial_reference());

    std::cout << "Adding existing dimensions to output view..." << std::endl;
    // Add the existing dimensions to the output point view
    // The points need to be processed in the order of the output view
    for (PtsStructs::PointId idx(0); idx < n_features; ++idx) {
        for (const auto &dim : predefined_dims) {
            las_distances_writer.points->copy_field<double>(
                dim, idx, las_reader.points, idx);
        }
        for (const auto &dim : proprietary_dims) {
            las_distances_writer.points->copy_field<double>(
                dim, idx, las_reader.points, idx);
        }

        PtsStructs::RayId ray_id = topo.get_ray_id(idx);
        const auto &ray = topo.get_ray(ray_id);
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

    // Gather single-echo rays and multi-echo rays separately to process them in
    // the right order
    auto ray_count = topo.ray_count();
    std::vector<PtsStructs::RayId> single_echo_ray_ids;
    std::vector<PtsStructs::RayId> multi_echo_ray_ids;
    for (PtsStructs::RayId ray_id(0); ray_id < ray_count; ++ray_id) {
        const auto &ray = topo.get_ray(ray_id);
        if (ray.empty()) {
            throw std::runtime_error("Empty ray.");
        }
        if (ray.get_number_of_returns() == 1) {
            single_echo_ray_ids.push_back(ray_id);
        } else {
            multi_echo_ray_ids.push_back(ray_id);
        }
    }

    // Timing counters
    auto time_multi_echo = std::chrono::microseconds(0);
    auto time_single_echo = std::chrono::microseconds(0);
    auto time_pca = std::chrono::microseconds(0);

    // Process multi-echo rays first to mark the points that are roof edges
    auto ray_count_multi = multi_echo_ray_ids.size();
    ProgressBarTotal bar_multi(
        ray_count_multi, "Processing multi-echo rays",
        indicators::option::ForegroundColor{indicators::Color::green});
    bar_multi.start();
    std::vector<bool> is_multi_echo_with_roof_edge(topo.ray_count(), false);
    for (PtsStructs::RayId ray_0_0_id : multi_echo_ray_ids) {
        const auto &ray_0_0 = topo.get_ray(ray_0_0_id);
        auto multi_start = std::chrono::high_resolution_clock::now();

        PtsStructs::PointId highest_return_idx =
            ray_0_0.get_point_id_in_return_order(-1);
        auto p_high = topo.points->get_point(highest_return_idx);

        for (PtsStructs::PointId p_0_0_id : ray_0_0.get_point_ids()) {
            double vertical_gain;

            auto p_0_0 = topo.points->get_point(p_0_0_id);

            vertical_gain = p_0_0.z() - p_high.z();
            bool is_roof_edge = vertical_gain > MIN_VERT_GAIN_ROOF;

            las_distances_writer.points->set_field(
                CustomDimensions::Id::VerticalGain, p_0_0_id, vertical_gain);
            las_distances_writer.points->set_field(
                CustomDimensions::Id::IsRoofEdge, p_0_0_id, is_roof_edge);

            // Add the points that are roof edges
            if (is_roof_edge) {
                const PtsStructs::ScanLineId scan_line_0_id =
                    topo.get_scan_line_id(ray_0_0_id);
                const std::optional<PtsStructs::ScanLineId> scan_line_p1_id =
                    topo.get_prev_scan_line_id(scan_line_0_id);
                const std::optional<PtsStructs::ScanLineId> scan_line_n1_id =
                    topo.get_next_scan_line_id(scan_line_0_id);
                const std::optional<PtsStructs::ScanLineId> scan_line_p2_id =
                    topo.get_prev_scan_line_id(scan_line_p1_id);
                const std::optional<PtsStructs::ScanLineId> scan_line_n2_id =
                    topo.get_next_scan_line_id(scan_line_n1_id);

                double best_planarity = 0.0;
                double best_horizontality = 0.0;

                const Direction directions[2] = {Direction::Previous,
                                                 Direction::Next};
                const std::optional<PtsStructs::ScanLineId> scan_line_1_ids[2] =
                    {scan_line_p1_id, scan_line_n1_id};
                const std::optional<PtsStructs::ScanLineId> scan_line_2_ids[2] =
                    {scan_line_p2_id, scan_line_n2_id};
                for (const auto &direction : directions) {
                    for (int i = 0; i < 2; ++i) {
                        auto planarity_and_horizontality =
                            compute_planarity_and_horizontality(
                                p_0_0_id, scan_line_1_ids[i],
                                scan_line_2_ids[i], direction, topo);
                        if (!planarity_and_horizontality) {
                            continue;
                        }
                        auto [planarity, horizontality] =
                            *planarity_and_horizontality;
                        if (planarity > best_planarity) {
                            best_planarity = planarity;
                            best_horizontality = horizontality;
                        }
                    }
                }

                PtsStructs::PointId edge_idx(
                    las_edge_writer.points->point_count());
                las_edge_writer.points->set_point(edge_idx, p_0_0);
                las_edge_writer.points->copy_field<int>(
                    pdal::Dimension::Id::Classification, edge_idx, topo.points,
                    p_0_0_id);
                uint8_t is_generated = 0;
                las_edge_writer.points->set_field(
                    CustomDimensions::Id::IsGenerated, edge_idx, is_generated);
                las_edge_writer.points->set_field(
                    CustomDimensions::Id::VerticalGain, edge_idx,
                    vertical_gain);
                las_edge_writer.points->set_field(
                    CustomDimensions::Id::Planarity, edge_idx, best_planarity);
                las_edge_writer.points->set_field(
                    CustomDimensions::Id::Horizontality, edge_idx,
                    best_horizontality);

                is_multi_echo_with_roof_edge[ray_0_0_id] = true;
            }
        }

        bar_multi.increment(1);

        auto multi_end = std::chrono::high_resolution_clock::now();
        time_multi_echo +=
            std::chrono::duration_cast<std::chrono::microseconds>(multi_end -
                                                                  multi_start);
    }
    bar_multi.finish();
    auto edge_count_multi = las_edge_writer.points->point_count();

    // Process single-echo rays
    auto ray_count_single = single_echo_ray_ids.size();
    ProgressBarTotal bar_single(
        ray_count_single, "Processing single-echo rays",
        indicators::option::ForegroundColor{indicators::Color::green});
    bar_single.start();

    for (PtsStructs::RayId ray_0_0_id : single_echo_ray_ids) {
        const auto &ray_0_0 = topo.get_ray(ray_0_0_id);

        auto single_start = std::chrono::high_resolution_clock::now();

        const auto scan_line_id = topo.get_scan_line_id(ray_0_0_id);
        const auto &scan_line = topo.get_scan_line(scan_line_id);

        const auto ray_0_p1_id = scan_line.get_prev_ray_id(ray_0_0_id);
        const auto ray_0_n1_id = scan_line.get_next_ray_id(ray_0_0_id);
        const auto ray_0_p2_id = scan_line.get_prev_ray_id(ray_0_p1_id);
        const auto ray_0_n2_id = scan_line.get_next_ray_id(ray_0_n1_id);

        PtsStructs::PointId p_0_0_id = ray_0_0.get_point_id_in_return_order(-1);
        Point_3 p_0_0 = topo.points->get_point(p_0_0_id);

        double vertical_gain = 0;
        Direction direction_with_max_gain;

        std::optional<PtsStructs::PointId> p_0_roof_1_id, p_0_roof_2_id,
            p_ground_id;

        // Check the neighbours in the previous and next rays for potential
        // ground points, but only if the same ray has not already given a roof
        // edge point

        if (ray_0_p1_id && !is_multi_echo_with_roof_edge[*ray_0_p1_id]) {
            PtsStructs::RayId ray_0_ground_id = *ray_0_p1_id;
            const auto &ray_0_ground = topo.get_ray(ray_0_ground_id);
            PtsStructs::PointId potential_p_ground_id =
                ray_0_ground.get_point_id_in_return_order(-1);
            Point_3 p_ground = topo.points->get_point(potential_p_ground_id);
            double vertical_gain_ = p_0_0.z() - p_ground.z();

            // If the vertical gain is better than the previous one
            if (vertical_gain_ > vertical_gain) {
                p_ground_id = potential_p_ground_id;
                vertical_gain = vertical_gain_;

                p_0_roof_1_id = find_closest_point(p_0_0_id, ray_0_n1_id, topo);
                p_0_roof_2_id =
                    find_closest_point(p_0_roof_1_id, ray_0_n2_id, topo);

                direction_with_max_gain = Direction::Previous;
            }
        }
        if (ray_0_n1_id && !is_multi_echo_with_roof_edge[*ray_0_n1_id]) {
            PtsStructs::RayId ray_0_ground_id = *ray_0_n1_id;
            const auto &ray_0_ground = topo.get_ray(ray_0_ground_id);
            PtsStructs::PointId potential_p_ground_id =
                ray_0_ground.get_point_id_in_return_order(-1);
            Point_3 p_ground = topo.points->get_point(potential_p_ground_id);
            double vertical_gain_ = p_0_0.z() - p_ground.z();

            // If the vertical gain is better than the previous one
            if (vertical_gain_ > vertical_gain) {
                p_ground_id = potential_p_ground_id;
                vertical_gain = vertical_gain_;

                p_0_roof_1_id = find_closest_point(p_0_0_id, ray_0_p1_id, topo);
                p_0_roof_2_id =
                    find_closest_point(p_0_roof_1_id, ray_0_p2_id, topo);

                direction_with_max_gain = Direction::Next;
            }
        }
        bool is_roof_edge = vertical_gain > MIN_VERT_GAIN_ROOF;

        las_distances_writer.points->set_field(
            CustomDimensions::Id::VerticalGain, p_0_0_id, vertical_gain);
        las_distances_writer.points->set_field(CustomDimensions::Id::IsRoofEdge,
                                               p_0_0_id, is_roof_edge);

        if (is_roof_edge) {
            // Add the points that are roof edges
            double gps_time = ray_0_0.get_gps_time();
            Point_3 p_edge;
            uint8_t is_generated;

            // Check whether we can extend the roof edge using the neighbours in
            // the same scan line
            bool extend_roof = false;
            if (p_0_roof_1_id && p_0_roof_2_id) {
                const Point_3 &p_roof_1 =
                    topo.points->get_point(*p_0_roof_1_id);
                const Point_3 &p_roof_2 =
                    topo.points->get_point(*p_0_roof_2_id);
                if (CustomCGAL::are_almost_collinear(
                        p_0_0, p_roof_1, p_roof_2,
                        CustomCGAL::Angle::from_degrees(5))) {
                    double distance_p_0_0_p_roof_1 =
                        std::sqrt(CGAL::squared_distance(p_0_0, p_roof_1));
                    double distance_p_roof_1_p_roof_2 =
                        std::sqrt(CGAL::squared_distance(p_roof_1, p_roof_2));
                    if (distance_p_0_0_p_roof_1 <
                            MAX_DISTANCE_NEIGHBOURS_ON_ROOF &&
                        distance_p_roof_1_p_roof_2 <
                            MAX_DISTANCE_NEIGHBOURS_ON_ROOF) {
                        extend_roof = true;
                    }
                }
            }

            // Create the roof edge point based on the chosen method
            if (extend_roof) {
                // Extension of the neighbours in the same scan line
                const Point_3 &p_roof_1 =
                    topo.points->get_point(*p_0_roof_1_id);
                const Point_3 &p_roof_2 =
                    topo.points->get_point(*p_0_roof_2_id);

                p_edge = p_0_0 + (p_0_0 - p_roof_1) / 2.0;

                is_generated = 1;
            } else {
                // Scanner position and middle point between the point and the
                // ground point
                if (!p_ground_id) {
                    throw std::runtime_error("No ground point found for "
                                             "single-echo point with ID " +
                                             std::to_string(p_0_0_id));
                }

                Point_3 p_ground = topo.points->get_point(*p_ground_id);
                Point_3 p_scanner = trajectory.get_point_at_gps_time(gps_time);
                Vector_3 scanner_to_p_0_0 = p_0_0 - p_scanner;
                Vector_3 scanner_to_ground = p_ground - p_scanner;
                Vector_3 scanner_to_edge =
                    (scanner_to_p_0_0 + scanner_to_ground) / 2.0;
                scanner_to_edge = scanner_to_edge /
                                  std::sqrt(scanner_to_edge.squared_length()) *
                                  std::sqrt(scanner_to_p_0_0.squared_length());

                p_edge = p_scanner + scanner_to_edge;
                is_generated = 2;
            }

            PtsStructs::PointId edge_idx(las_edge_writer.points->point_count());
            las_edge_writer.points->set_point(edge_idx, p_edge);
            las_edge_writer.points
                ->copy_field<std::underlying_type_t<LASclassification::Value>>(
                    pdal::Dimension::Id::Classification, edge_idx, topo.points,
                    p_0_0_id);
            las_edge_writer.points->set_field(CustomDimensions::Id::IsGenerated,
                                              edge_idx, is_generated);
            las_edge_writer.points->set_field(
                CustomDimensions::Id::VerticalGain, edge_idx, vertical_gain);

            // Compute the roof likelihood as two measures: the planarity and
            // the horizontality
            auto pca_start = std::chrono::high_resolution_clock::now();
            const auto scan_line_p1_id =
                topo.get_prev_scan_line_id(scan_line_id);
            const auto scan_line_n1_id =
                topo.get_next_scan_line_id(scan_line_id);
            const auto scan_line_p2_id =
                topo.get_prev_scan_line_id(scan_line_p1_id);
            const auto scan_line_n2_id =
                topo.get_next_scan_line_id(scan_line_n1_id);

            const std::optional<PtsStructs::ScanLineId> scan_line_1_ids[2] = {
                scan_line_p1_id, scan_line_n1_id};
            const std::optional<PtsStructs::ScanLineId> scan_line_2_ids[2] = {
                scan_line_p2_id, scan_line_n2_id};

            double best_planarity = 0.0;
            double best_horizontality = 0.0;
            for (int i = 0; i < 2; ++i) {
                auto planarity_and_horizontality =
                    compute_planarity_and_horizontality(
                        p_0_0_id, scan_line_1_ids[i], scan_line_2_ids[i],
                        direction_with_max_gain, topo);
                if (!planarity_and_horizontality) {
                    continue;
                }
                auto [planarity, horizontality] = *planarity_and_horizontality;
                if (planarity > best_planarity) {
                    best_planarity = planarity;
                    best_horizontality = horizontality;
                }
            }
            auto pca_end = std::chrono::high_resolution_clock::now();
            time_pca += std::chrono::duration_cast<std::chrono::microseconds>(
                pca_end - pca_start);

            las_edge_writer.points->set_field(CustomDimensions::Id::Planarity,
                                              edge_idx, best_planarity);
            las_edge_writer.points->set_field(
                CustomDimensions::Id::Horizontality, edge_idx,
                best_horizontality);
        }
        bar_single.increment(1);

        auto single_end = std::chrono::high_resolution_clock::now();
        time_single_echo +=
            std::chrono::duration_cast<std::chrono::microseconds>(single_end -
                                                                  single_start);
    }
    bar_single.finish();

    auto edge_count_single =
        las_edge_writer.points->point_count() - edge_count_multi;

    // Print timing breakdown
    std::cout << "\n=== Timing Breakdown ===" << std::endl;
    std::cout << "Multi-echo rays (with edge): " << ray_count_multi
              << " rays (total: " << time_multi_echo.count() / 1e6 << "s)"
              << std::endl;
    std::cout << "Single-echo rays (with edge): " << ray_count_single
              << " rays (total: " << time_single_echo.count() / 1e6 << "s)"
              << std::endl;
    std::cout << "  - PCA computation: " << time_pca.count() << "μs"
              << std::endl;
    std::cout << "========================\n" << std::endl;

    std::cout << "Number of multi-echo rays giving a roof edge: "
              << edge_count_multi << std::endl;
    std::cout << "Number of single-echo rays giving a roof edge: "
              << edge_count_single << std::endl;

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