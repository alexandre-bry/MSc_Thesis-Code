#include "distances.hpp"

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

#include "../geom/cgal.hpp"
#include "../geom/points.hpp"
#include "../las/reader.hpp"
#include "../las/trajectory.hpp"
#include "../las/writer.hpp"
#include "../pca/pca.hpp"
#include "../utils/pbar.hpp"

const double MIN_VERT_GAIN_ROOF = 2.0;
const double DEFAULT_GPS_TIME_DELTA = 1e-6;
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

/**
 * @brief Computes the inward direction for a given point.
 * The formula is expected to work well for points on roof edges, because it
 * takes into account all the points in a 3D neighbourhood. For façade points
 * close to the ground, it will point outwards instead of inward, and for façade
 * points which have a very vertical neighbourhood, it should give a very small
 * inward direction, which is good because we want to avoid using those points.
 *
 * @param focus_point_id The point for which to compute the inward direction
 * @param storage The point storage containing the points and the KD-tree
 * @param point_ids_scratch A scratch vector to store the point IDs of the
 * neighbours
 * @param inward_direction The computed inward direction (output)
 */
void compute_inward_direction_roof(PtsStructs::PointId focus_point_id,
                                   PtsStructs::StoragePtr storage,
                                   std::vector<std::size_t> &point_ids_scratch,
                                   Vector_3 &inward_direction) {
    const double NEIGHBOURS_RADIUS = 2.0;
    const double MAX_Z_DIFF = 1.0;

    const auto kd_tree_3 = storage->get_kd_tree_3d();

    point_ids_scratch.clear();
    const Point_3 &focus_point = storage->get_point(focus_point_id);
    kd_tree_3->search_indices_in_sphere(focus_point, NEIGHBOURS_RADIUS,
                                        point_ids_scratch);

    Vector_2 weighted_inward_dir(0.0, 0.0);
    double weights = 0.0;
    for (const auto &point_id : point_ids_scratch) {
        if (focus_point_id == point_id) {
            continue;
        }
        const Point_3 &point =
            storage->get_point(PtsStructs::PointId(point_id));
        if (std::abs(point.z() - focus_point.z()) > MAX_Z_DIFF) {
            continue;
        }

        UnitVector_2 inward_dir(Point_2(point.x(), point.y()) -
                                Point_2(focus_point.x(), focus_point.y()));

        weights += 1.0;
        weighted_inward_dir += inward_dir;
    }
    if (weights > 0) {
        weighted_inward_dir /= weights;
    } else {
        weighted_inward_dir = Vector_2(0.0, 0.0);
    }

    inward_direction =
        Vector_3(weighted_inward_dir.x(), weighted_inward_dir.y(), 0.0);
}

/**
 * @brief Computes the inward direction for a given point.
 * The formula is expected to work well for points on façades, as it takes into
 * account all the points in a 2D neighbourhood that are below on the Z axis.
 *
 * @param focus_point_id The point for which to compute the inward direction
 * @param storage The point storage containing the points and the KD-tree
 * @param point_ids_scratch A scratch vector to store the point IDs of the
 * neighbours
 * @param ground_point_mask A mask to indicate which points are classified as
 * ground points
 * @param inward_direction The computed inward direction (output)
 */
void compute_inward_direction_facade(
    PtsStructs::PointId focus_point_id, PtsStructs::StoragePtr storage,
    std::vector<std::size_t> &point_ids_scratch,
    const std::vector<bool> &ground_point_mask, Vector_3 &inward_direction) {
    const double NEIGHBOURS_RADIUS = 2.0;
    const double Z_BUFFER = 0.1;

    point_ids_scratch.clear();
    const auto kd_tree_2 = storage->get_kd_tree_2d();

    const Point_3 &focus_point_3d = storage->get_point(focus_point_id);
    const Point_2 focus_point_2d(focus_point_3d.x(), focus_point_3d.y());
    kd_tree_2->search_indices_in_circle(focus_point_2d, NEIGHBOURS_RADIUS,
                                        point_ids_scratch);

    Vector_2 weighted_inward_dir(0.0, 0.0);
    double weights = 0.0;
    for (const auto &point_id : point_ids_scratch) {
        if (focus_point_id == point_id) {
            continue;
        }
        // if (!ground_point_mask[point_id]) {
        //     continue;
        // }
        const Point_3 &point_3d =
            storage->get_point(PtsStructs::PointId(point_id));
        double height_diff = focus_point_3d.z() - point_3d.z();
        double weight = height_diff;

        UnitVector_2 inward_dir(focus_point_2d -
                                Point_2(point_3d.x(), point_3d.y()));

        weights += std::abs(weight);
        weighted_inward_dir += weight * inward_dir;
    }
    if (weights > 0) {
        weighted_inward_dir /= weights;
    } else {
        weighted_inward_dir = Vector_2(0.0, 0.0);
    }

    inward_direction =
        Vector_3(weighted_inward_dir.x(), weighted_inward_dir.y(), 0.0);
}

const std::set<std::string> valid_inward_types = {"roof", "facade"};

void compute_inward_directions(const std::string &input_points_file,
                               const std::string &output_points_file,
                               const std::string &type, bool overwrite) {
    if (std::filesystem::exists(output_points_file) && !overwrite) {
        throw std::runtime_error("Output file already exists: " +
                                 output_points_file);
    }

    if (valid_inward_types.find(type) == valid_inward_types.end()) {
        std::string valid_types = "(";
        for (const auto &valid_type : valid_inward_types) {
            valid_types += valid_type + ", ";
        }
        valid_types = valid_types.substr(0, valid_types.size() - 2) + ")";
        throw std::runtime_error("Invalid type of inward direction: " + type +
                                 ". Valid types are: " + valid_types);
    }
    /* ---------------- Read the input and prepare the output --------------- */

    // Read the LAS file and get the point view
    std::cout << "Reading LAS file..." << std::endl;
    LasReader las_reader(input_points_file);
    auto [predefined_dims, proprietary_dims] = las_reader.points->dimensions();
    auto n_features = las_reader.points->point_count();

    std::cout << "Number of points: " << n_features << std::endl;

    // Prepare the topology
    std::cout << "Preparing topology..." << std::endl;
    PtsStructs::StoragePtr storage_ptr(las_reader.points);

    // Prepare the output writer
    std::cout << "Preparing output object..." << std::endl;
    std::vector<pdal::Dimension::Id> distances_dims = predefined_dims;
    std::vector<ProprietaryDimension> distances_custom_dims = proprietary_dims;
    std::vector<ProprietaryDimension> new_distances_custom_dims = {
        CustomDimensions::Id::InwardVectorX,
        CustomDimensions::Id::InwardVectorY,
        CustomDimensions::Id::InwardVectorZ,
    };
    distances_custom_dims.insert(distances_custom_dims.end(),
                                 new_distances_custom_dims.begin(),
                                 new_distances_custom_dims.end());
    LasWriter las_writer(distances_dims, distances_custom_dims,
                         las_reader.points->spatial_reference());

    // Add the existing dimensions to the output point view
    std::cout << "Adding existing dimensions to output view..." << std::endl;
    ProgressBarTotal pbar_add_dims(n_features, "Adding existing dimensions");
    for (PtsStructs::PointId idx(0); idx < n_features; ++idx) {
        for (const auto &dim : predefined_dims) {
            las_writer.points->copy_field<double>(dim, idx, las_reader.points,
                                                  idx);
        }
        for (const auto &dim : proprietary_dims) {
            las_writer.points->copy_field<double>(dim, idx, las_reader.points,
                                                  idx);
        }

        pbar_add_dims.increment(1);
    }
    pbar_add_dims.finish();

    /* ------------------------- Process the points ------------------------- */

    if (type == "roof") {
        storage_ptr->build_kd_tree_3d();
    } else if (type == "facade") {
        storage_ptr->build_kd_tree_2d();
    } else {
        throw std::runtime_error("Invalid inward direction type: " + type);
    }

    // Cache the ground point mask if needed for the façade inward direction
    std::vector<bool> ground_point_mask;
    if (type == "facade") {
        std::cout << "Caching ground point mask..." << std::endl;
        ground_point_mask.resize(n_features);
        for (PtsStructs::PointId idx(0); idx < n_features; ++idx) {
            const auto cls_raw = las_reader.points->get_field_as<
                std::underlying_type_t<LASclassification::Value>>(
                pdal::Dimension::Id::Classification, idx);
            const auto cls = static_cast<LASclassification::Value>(cls_raw);
            ground_point_mask[idx] = (cls == LASclassification::Value::Ground);
        }
    }

    // Compute the inward directions for each point
    std::cout << "Computing inward directions for each point..." << std::endl;
    ProgressBarTotal pbar_inward(n_features, "Computing inward directions");
    std::vector<std::size_t> point_ids_scratch;
    point_ids_scratch.reserve(64);

    for (PtsStructs::PointId idx(0); idx < n_features; ++idx) {
        Vector_3 inward_direction;
        if (type == "roof") {
            compute_inward_direction_roof(idx, storage_ptr, point_ids_scratch,
                                          inward_direction);
        } else if (type == "facade") {
            compute_inward_direction_facade(idx, storage_ptr, point_ids_scratch,
                                            ground_point_mask,
                                            inward_direction);
        } else {
            throw std::runtime_error("Invalid inward direction type: " + type);
        }
        las_writer.points->set_field(CustomDimensions::Id::InwardVectorX, idx,
                                     inward_direction.x());
        las_writer.points->set_field(CustomDimensions::Id::InwardVectorY, idx,
                                     inward_direction.y());
        las_writer.points->set_field(CustomDimensions::Id::InwardVectorZ, idx,
                                     inward_direction.z());

        pbar_inward.increment(1);
    }
    pbar_inward.finish();

    /* -------------------------- Write the output -------------------------- */

    // Write the output LAS file
    std::cout << "Writing output LAS file..." << std::endl;
    las_writer.write(output_points_file, {});
}

void identify_roof_edge_points(const std::string &input_points_file,
                               const std::string &input_trajectory_file,
                               const std::string &output_distances_file,
                               const std::string &output_edges_file,
                               bool overwrite) {
    const double ANGLE_BUFFER_FACTOR =
        1.0; // Assumes a 45 degree angle for the roof

    /* ---------------------------------------------------------------------- */
    /*                            Check input files                           */
    /* ---------------------------------------------------------------------- */

    if (std::filesystem::exists(output_distances_file) && !overwrite) {
        throw std::runtime_error("Output file already exists: " +
                                 output_distances_file);
    } else if (std::filesystem::exists(output_edges_file) && !overwrite) {
        throw std::runtime_error("Output file already exists: " +
                                 output_edges_file);
    }

    /* ---------------------------------------------------------------------- */
    /*                      Read input and prepare output                     */
    /* ---------------------------------------------------------------------- */

    // Read the LAS file and get the point view
    std::cout << "Reading LAS file..." << std::endl;
    LasReader las_reader(input_points_file);
    auto [predefined_dims, proprietary_dims] = las_reader.points->dimensions();
    auto n_features = las_reader.points->point_count();

    std::cout << "Number of points: " << n_features << std::endl;

    // Prepare the trajectory
    std::cout << "Reading trajectory file..." << std::endl;
    Trajectory trajectory = read_trajectory(input_trajectory_file);

    // Prepare the points with attributes
    PtsStructs::Topology3D topo(las_reader.points, trajectory);
    topo.points->build_kd_tree_3d();

    // Prepare the output writers
    std::cout << "Preparing output objects..." << std::endl;
    std::vector<pdal::Dimension::Id> distances_dims = predefined_dims;
    std::vector<ProprietaryDimension> distances_custom_dims = proprietary_dims;
    std::vector<ProprietaryDimension> new_distances_custom_dims = {
        CustomDimensions::Id::ReturnNumberComputed,
        CustomDimensions::Id::NumberOfReturnsComputed,
        CustomDimensions::Id::MaxVerticalDiff,
        CustomDimensions::Id::MinVerticalDiff,
        CustomDimensions::Id::IsRoofEdge,
        CustomDimensions::Id::IsFacade,
        CustomDimensions::Id::ScannerPositionX,
        CustomDimensions::Id::ScannerPositionY,
        CustomDimensions::Id::ScannerPositionZ,
    };
    distances_custom_dims.insert(distances_custom_dims.end(),
                                 new_distances_custom_dims.begin(),
                                 new_distances_custom_dims.end());
    LasWriter las_distances_writer(distances_dims, distances_custom_dims,
                                   las_reader.points->spatial_reference());

    std::vector<pdal::Dimension::Id> edge_dims = {
        pdal::Dimension::Id::X,
        pdal::Dimension::Id::Y,
        pdal::Dimension::Id::Z,
        pdal::Dimension::Id::Classification,
    };
    std::vector<ProprietaryDimension> edge_custom_dims = {
        CustomDimensions::Id::IsGenerated,
        CustomDimensions::Id::MaxVerticalDiff,
        CustomDimensions::Id::InwardVectorX,
        CustomDimensions::Id::InwardVectorY,
        CustomDimensions::Id::InwardVectorZ,
    };
    LasWriter las_edge_writer(edge_dims, edge_custom_dims,
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
        const Point_3 &point_scanner =
            trajectory.get_pos_at_gps_time(ray.get_gps_time());

        las_distances_writer.points->set_field(
            CustomDimensions::Id::ReturnNumberComputed, idx,
            return_number_computed);
        las_distances_writer.points->set_field(
            CustomDimensions::Id::NumberOfReturnsComputed, idx,
            number_of_returns_computed);
        las_distances_writer.points->set_field(
            CustomDimensions::Id::ScannerPositionX, idx, point_scanner.x());
        las_distances_writer.points->set_field(
            CustomDimensions::Id::ScannerPositionY, idx, point_scanner.y());
        las_distances_writer.points->set_field(
            CustomDimensions::Id::ScannerPositionZ, idx, point_scanner.z());
    }

    std::cout << "Computing distances..." << std::endl;

    // Gather single-echo rays and multi-echo rays separately to process
    // them in the right order
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

    std::vector<bool> is_facade_point(topo.points->point_count(), false);

    /* ---------------------------------------------------------------------- */
    /*                         Process multi-echo rays                        */
    /* ---------------------------------------------------------------------- */

    // We do multi-echo first to mark them as roof edges and avoid using
    // them for the single-echo rays
    auto ray_count_multi = multi_echo_ray_ids.size();
    ProgressBarTotal bar_multi(
        ray_count_multi, "Processing multi-echo rays",
        indicators::option::ForegroundColor{indicators::Color::green});
    bar_multi.start();
    std::vector<bool> is_multi_echo_with_roof_edge(topo.ray_count(), false);
    for (PtsStructs::RayId ray_0_0_id : multi_echo_ray_ids) {
        const auto &ray_0_0 = topo.get_ray(ray_0_0_id);

        // Extract the point corresponding to the last return, which is the one
        // that could be ground
        PtsStructs::PointId last_return_idx =
            ray_0_0.get_point_id_in_return_order(-1);
        auto p_last_return = topo.points->get_point(last_return_idx);

        // Check the vertical gain of the other returns compared to the last
        for (PtsStructs::PointId p_0_0_id : ray_0_0.get_point_ids()) {
            auto p_0_0 = topo.points->get_point(p_0_0_id);

            double vertical_gain = p_0_0.z() - p_last_return.z();
            bool is_roof_edge = vertical_gain > MIN_VERT_GAIN_ROOF;

            // Mark the last return as a facade point if one of the previous
            // returns is a roof edge
            if (is_roof_edge) {
                auto raw_cls = static_cast<LASclassification::Value>(
                    topo.points->get_field_as<uint8_t>(
                        pdal::Dimension::Id::Classification, p_0_0_id));
                if (raw_cls != LASclassification::Value::LowVegetation &&
                    raw_cls != LASclassification::Value::MediumVegetation &&
                    raw_cls != LASclassification::Value::HighVegetation) {
                    is_facade_point[last_return_idx] = true;
                }
            }
            las_distances_writer.points->set_field(
                CustomDimensions::Id::MaxVerticalDiff, p_0_0_id, vertical_gain);
            las_distances_writer.points->set_field(
                CustomDimensions::Id::IsRoofEdge, p_0_0_id, is_roof_edge);

            // Add the points that are roof edges
            if (is_roof_edge) {
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
                    CustomDimensions::Id::MaxVerticalDiff, edge_idx,
                    vertical_gain);
                las_edge_writer.points->copy_field<double>(
                    CustomDimensions::Id::InwardVectorX, edge_idx, topo.points,
                    p_0_0_id);
                las_edge_writer.points->copy_field<double>(
                    CustomDimensions::Id::InwardVectorY, edge_idx, topo.points,
                    p_0_0_id);
                las_edge_writer.points->copy_field<double>(
                    CustomDimensions::Id::InwardVectorZ, edge_idx, topo.points,
                    p_0_0_id);

                is_multi_echo_with_roof_edge[ray_0_0_id] = true;
            }
        }

        bar_multi.increment(1);
    }
    bar_multi.finish();
    auto edge_count_multi = las_edge_writer.points->point_count();

    /* ---------------------------------------------------------------------- */
    /*                        Process single-echo rays                        */
    /* ---------------------------------------------------------------------- */

    auto ray_count_single = single_echo_ray_ids.size();
    ProgressBarTotal bar_single(
        ray_count_single, "Processing single-echo rays",
        indicators::option::ForegroundColor{indicators::Color::green});
    bar_single.start();

    for (PtsStructs::RayId ray_0_0_id : single_echo_ray_ids) {
        const auto &ray_0_0 = topo.get_ray(ray_0_0_id);

        /* --------------------- Find all the neighbours -------------------- */

        const auto scan_line_id = topo.get_scan_line_id(ray_0_0_id);
        const auto prev_scan_line_id = topo.get_prev_scan_line_id(scan_line_id);
        const auto next_scan_line_id = topo.get_next_scan_line_id(scan_line_id);

        const auto &scan_line = topo.get_scan_line(scan_line_id);

        std::vector<std::optional<PtsStructs::RayId>> neighbour_ray_ids = {
            scan_line.get_prev_ray_id(ray_0_0_id),
            scan_line.get_next_ray_id(ray_0_0_id),

        };
        if (prev_scan_line_id) {
            const auto &prev_scan_line = topo.get_scan_line(*prev_scan_line_id);
            neighbour_ray_ids.push_back(
                prev_scan_line.get_closest_ray_by_direction(ray_0_0_id));
        }
        if (next_scan_line_id) {
            const auto &next_scan_line = topo.get_scan_line(*next_scan_line_id);
            neighbour_ray_ids.push_back(
                next_scan_line.get_closest_ray_by_direction(ray_0_0_id));
        }

        // Get the only point of the ray
        PtsStructs::PointId p_0_0_id = ray_0_0.get_point_id_in_return_order(0);
        LASclassification::Value raw_cls =
            static_cast<LASclassification::Value>(
                topo.points->get_field_as<uint8_t>(
                    pdal::Dimension::Id::Classification, p_0_0_id));
        // We skip points classified as vegetation
        if (raw_cls == LASclassification::Value::LowVegetation ||
            raw_cls == LASclassification::Value::MediumVegetation ||
            raw_cls == LASclassification::Value::HighVegetation) {
            bar_single.increment(1);

            las_distances_writer.points->set_field(
                CustomDimensions::Id::MaxVerticalDiff, p_0_0_id, 0.0);
            las_distances_writer.points->set_field(
                CustomDimensions::Id::IsRoofEdge, p_0_0_id, 0.0);

            continue;
        }
        Point_3 p_0_0 = topo.points->get_point(p_0_0_id);

        double max_vertical_diff = 0.0;
        double min_vertical_diff = 0.0;

        std::optional<PtsStructs::PointId> p_0_roof_1_id, p_0_roof_2_id,
            p_ground_id;
        std::optional<PtsStructs::RayId> ray_ground_id;

        // Check the neighbour rays only if they have not already been marked as
        // having a roof edge point

        for (const auto &neighbour_ray_id : neighbour_ray_ids) {
            if (!neighbour_ray_id) {
                continue;
            }
            // If the neighbour ray has already given a roof edge point, we
            // skip it because we assume that it was a better candidate
            if (is_multi_echo_with_roof_edge[*neighbour_ray_id]) {
                continue;
            }

            // Get the last return of the neighbour ray as a potential ground
            // point
            PtsStructs::RayId potential_ray_ground_id = *neighbour_ray_id;
            const auto &ground_ray = topo.get_ray(potential_ray_ground_id);
            PtsStructs::PointId potential_p_ground_id =
                ground_ray.get_point_id_in_return_order(-1);
            Point_3 p_ground = topo.points->get_point(potential_p_ground_id);
            double vertical_diff_current = p_0_0.z() - p_ground.z();

            // If the vertical gain is better than the previous one
            if (vertical_diff_current > max_vertical_diff) {
                // We want to prevent matching points that are simply far from
                // each other on the same roof slope, so we remove from the
                // vertical gain a height proportional to the horizontal
                // distance between the two rays at the height of the point
                const Point_3 &p_ground_at_p_0_0_height =
                    topo.get_point_at_height(potential_ray_ground_id,
                                             p_0_0.z());
                double horizontal_distance = std::sqrt(
                    CGAL::squared_distance(p_0_0, p_ground_at_p_0_0_height));
                vertical_diff_current -=
                    ANGLE_BUFFER_FACTOR * horizontal_distance;

                if (max_vertical_diff > vertical_diff_current) {
                    continue;
                }

                // p_0_roof_1_id = find_closest_point(p_0_0_id, ray_0_n1_id,
                // topo); p_0_roof_2_id =
                //     find_closest_point(p_0_roof_1_id, ray_0_n2_id, topo);

                p_ground_id = potential_p_ground_id;
                ray_ground_id = potential_ray_ground_id;
                max_vertical_diff = vertical_diff_current;
            }
        }

        // Decide whether the point is a roof edge based on the vertical
        // gain and the GPS time delta to the potential ground point
        bool is_roof_edge = max_vertical_diff > MIN_VERT_GAIN_ROOF;

        if (is_roof_edge) {
            if (p_ground_id) {
                is_facade_point[*p_ground_id] = true;
            } else {
                std::cerr
                    << "Warning: No ground point found for single-echo point "
                    << "with ID " << p_0_0_id << " and ray ID " << ray_0_0_id
                    << std::endl;
            }
        }

        las_distances_writer.points->set_field(
            CustomDimensions::Id::MaxVerticalDiff, p_0_0_id, max_vertical_diff);
        las_distances_writer.points->set_field(CustomDimensions::Id::IsRoofEdge,
                                               p_0_0_id, is_roof_edge);

        if (is_roof_edge) {
            // Add the points that are roof edges
            double gps_time = ray_0_0.get_gps_time();
            Point_3 p_edge;
            uint8_t is_generated;

            // Check whether we can extend the roof edge using the
            // neighbours in the same scan line
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
                // Scanner position and middle point between the point and
                // the ground point
                if (!p_ground_id) {
                    throw std::runtime_error("No ground point found for "
                                             "single-echo point with ID " +
                                             std::to_string(p_0_0_id));
                }

                // Point_3 p_ground = topo.points->get_point(*p_ground_id);
                // Point_3 p_scanner =
                // trajectory.get_point_at_gps_time(gps_time); Vector_3
                // scanner_to_p_0_0 = p_0_0 - p_scanner; Vector_3
                // scanner_to_ground = p_ground - p_scanner; Vector_3
                // scanner_to_edge =
                //     (scanner_to_p_0_0 + scanner_to_ground) / 2.0;
                // scanner_to_edge = scanner_to_edge /
                //                   std::sqrt(scanner_to_edge.squared_length())
                //                   *
                //                   std::sqrt(scanner_to_p_0_0.squared_length());
                // p_edge = p_scanner + scanner_to_edge;
                const Point_3 &p_ground_at_p_0_0_height =
                    topo.get_point_at_height(*ray_ground_id, p_0_0.z());
                p_edge =
                    CGAL::ORIGIN + ((p_0_0 - CGAL::ORIGIN) +
                                    (p_ground_at_p_0_0_height - CGAL::ORIGIN)) /
                                       2.0;

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
                CustomDimensions::Id::MaxVerticalDiff, edge_idx,
                max_vertical_diff);

            las_edge_writer.points->copy_field<double>(
                CustomDimensions::Id::InwardVectorX, edge_idx, topo.points,
                p_0_0_id);
            las_edge_writer.points->copy_field<double>(
                CustomDimensions::Id::InwardVectorY, edge_idx, topo.points,
                p_0_0_id);
            las_edge_writer.points->copy_field<double>(
                CustomDimensions::Id::InwardVectorZ, edge_idx, topo.points,
                p_0_0_id);
        }
        bar_single.increment(1);
    }
    bar_single.finish();

    auto edge_count_single =
        las_edge_writer.points->point_count() - edge_count_multi;

    std::cout << "Number of multi-echo rays giving a roof edge: "
              << edge_count_multi << std::endl;
    std::cout << "Number of single-echo rays giving a roof edge: "
              << edge_count_single << std::endl;

    for (PtsStructs::PointId idx(0); idx < topo.points->point_count(); ++idx) {
        las_distances_writer.points->set_field(
            CustomDimensions::Id::IsFacade, idx,
            static_cast<uint8_t>(is_facade_point[idx]));
    }

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