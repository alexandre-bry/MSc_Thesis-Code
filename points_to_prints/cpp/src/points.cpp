#include "points.hpp"

#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>

#include "utils/cgal.hpp"
#include "utils/kd_tree.hpp"

using namespace PtsStructs;

void Storage::init(std::vector<pdal::Dimension::Id> predefined_dims,
                   std::vector<ProprietaryDimension> proprietary_dims,
                   pdal::SpatialReference spatial_ref) {
    table = std::make_shared<pdal::PointTable>();

    for (auto dim : predefined_dims) {
        table->layout()->registerDim(dim);
    }
    for (auto dim : proprietary_dims) {
        table->layout()->registerOrAssignDim(dim.name, dim.type);
    }

    table->clearSpatialReferences();
    table->setSpatialReference(spatial_ref);

    view = pdal::PointViewPtr(new pdal::PointView(*table));
}

Storage::Storage(std::vector<pdal::Dimension::Id> predefined_dims,
                 std::vector<ProprietaryDimension> proprietary_dims,
                 pdal::SpatialReference spatial_ref) {
    init(predefined_dims, proprietary_dims, spatial_ref);
}

Storage::Storage(pdal::PointViewPtr input_view,
                 pdal::SpatialReference spatial_ref) {
    pdal::Dimension::IdList pdal_dims = input_view->dims();
    std::vector<pdal::Dimension::Id> predefined_dims;
    std::vector<ProprietaryDimension> proprietary_dims;
    for (auto dim : pdal_dims) {
        if (static_cast<int>(dim) >= pdal::Dimension::PROPRIETARY) {
            proprietary_dims.push_back(ProprietaryDimension(
                input_view->dimName(dim), input_view->dimType(dim)));
        } else {
            predefined_dims.push_back(dim);
        }
    }

    init(predefined_dims, proprietary_dims, spatial_ref);
}

Storage::Storage(pdal::PointViewPtr view,
                 std::shared_ptr<pdal::PointTable> table)
    : view(view), table(std::move(table)) {}

std::size_t Storage::point_count() const { return view->size(); }

pdal::SpatialReference Storage::spatial_reference() const {
    return view->spatialReference();
}

std::pair<std::vector<pdal::Dimension::Id>, std::vector<ProprietaryDimension>>
Storage::dimensions() const {
    pdal::Dimension::IdList pdal_dims = view->dims();
    std::vector<pdal::Dimension::Id> predefined_dims;
    std::vector<ProprietaryDimension> proprietary_dims;
    for (auto dim : pdal_dims) {
        if (static_cast<int>(dim) >= pdal::Dimension::PROPRIETARY) {
            proprietary_dims.push_back(
                ProprietaryDimension(view->dimName(dim), view->dimType(dim)));
        } else {
            predefined_dims.push_back(dim);
        }
    }
    return {predefined_dims, proprietary_dims};
}

OGREnvelopePtr Storage::bounding_box() const {
    pdal::BOX2D bounds;
    view->calculateBounds(bounds);
    OGREnvelopePtr env(new OGREnvelope());
    env->MinX = bounds.minx;
    env->MaxX = bounds.maxx;
    env->MinY = bounds.miny;
    env->MaxY = bounds.maxy;
    return env;
}

Bbox_2 Storage::bounding_box_cgal() const {
    pdal::BOX2D bounds;
    view->calculateBounds(bounds);
    return Bbox_2(bounds.minx, bounds.miny, bounds.maxx, bounds.maxy);
}

void Storage::build_kd_tree_2d() {
    if (las_kd_tree) {
        return; // Kd-tree already built
    }
    std::vector<Point_2> point_2d;
    point_2d.reserve(view->size());
    for (pdal::PointId i = 0; i < view->size(); ++i) {
        double x = view->getFieldAs<double>(pdal::Dimension::Id::X, i);
        double y = view->getFieldAs<double>(pdal::Dimension::Id::Y, i);
        point_2d.emplace_back(x, y);
    }
    las_kd_tree = std::make_shared<KdTree_2>(point_2d);
}

std::shared_ptr<KdTree_2> Storage::get_kd_tree_2d() const {
    if (las_kd_tree) {
        return las_kd_tree;
    }
    throw std::runtime_error("Kd-tree not built");
}

Ray3D::Ray3D(const Point_3 &origin_, double gps_time_,
             uint8_t scan_direction_flag_, double scan_angle_,
             const std::vector<PointId> &point_ids_,
             const std::vector<int> &return_numbers)
    : origin(origin_), gps_time(gps_time_),
      scan_direction_flag(scan_direction_flag_), scan_angle(scan_angle_) {
    // Sort the point IDs by return number
    std::vector<std::pair<PointId, int>> point_id_and_return_number;
    for (size_t i = 0; i < point_ids_.size(); ++i) {
        PointId point_id = point_ids_[i];
        int return_number = return_numbers[i];
        point_id_and_return_number.push_back(
            std::make_pair(point_id, return_number));
    }

    // Sort the point IDs by return number
    std::sort(
        point_id_and_return_number.begin(), point_id_and_return_number.end(),
        [](const std::pair<PointId, int> &a, const std::pair<PointId, int> &b) {
            return a.second < b.second;
        });

    for (const auto &pair : point_id_and_return_number) {
        return_number_to_point_id.push_back(pair.first);
        point_id_to_return_number[pair.first] = pair.second;
    }
}

PointId Ray3D::get_point_id_in_return_order(int return_number) const {
    // Handle the case where the index is smaller than 0
    auto number_of_returns = this->get_number_of_returns();
    if (return_number < 0) {
        return_number = number_of_returns + return_number;
    }

    // Check if out of bounds
    if (return_number < 0 || return_number >= number_of_returns) {
        throw std::out_of_range("Return number is out of bounds");
    }

    // Find the point with the given return number
    return return_number_to_point_id[return_number];
}

CustomCGAL::Angle Ray3D::angle_to(const Ray3D &other, StoragePtr points) const {
    auto v1 =
        this->origin - points->get_point(this->get_point_id_in_return_order(0));
    auto v2 =
        other.origin - points->get_point(other.get_point_id_in_return_order(0));
    CustomCGAL::Angle angle = CustomCGAL::angle(v1, v2);
    if (angle.in_degrees() > 180) {
        angle = CustomCGAL::Angle::from_degrees(360 - angle.in_degrees());
    }
    return angle;
}

ScanLine3D::ScanLine3D(StoragePtr points_,
                       std::shared_ptr<std::vector<Ray3D>> rays_,
                       const std::vector<RayId> &ray_ids_)
    : points(points_), rays(rays_), ray_ids(ray_ids_) {
    for (size_t i = 0; i < ray_ids.size(); ++i) {
        ray_id_to_index[ray_ids[i]] = i;
        PointId point_id = rays->at(ray_ids[i]).get_point_id_in_return_order(0);

        double scan_angle = points->get_field_as<double>(
            pdal::Dimension::Id::ScanAngleRank, point_id);
        scan_angle_to_ray_id[scan_angle] = ray_ids[i];
    }
}

std::optional<RayId>
ScanLine3D::get_next_ray_id(std::optional<RayId> ray_id) const {
    if (!ray_id) {
        return std::nullopt;
    }
    auto it = ray_id_to_index.find(*ray_id);
    if (it == ray_id_to_index.end()) {
        return std::nullopt;
    }
    size_t index = it->second;
    if (index + 1 < ray_ids.size()) {
        return ray_ids[index + 1];
    } else {
        return std::nullopt;
    }
}
std::optional<RayId>
ScanLine3D::get_prev_ray_id(std::optional<RayId> ray_id) const {
    if (!ray_id) {
        return std::nullopt;
    }
    auto it = ray_id_to_index.find(*ray_id);
    if (it == ray_id_to_index.end()) {
        return std::nullopt;
    }
    size_t index = it->second;
    if (index > 0) {
        return ray_ids[index - 1];
    } else {
        return std::nullopt;
    }
}

RayId ScanLine3D::get_closest_ray_by_scan_angle(double scan_angle) const {
    auto it = scan_angle_to_ray_id.lower_bound(scan_angle);
    if (it == scan_angle_to_ray_id.end()) {
        return ray_ids.back();
    }
    if (it == scan_angle_to_ray_id.begin()) {
        return ray_ids.front();
    }
    auto next_it = it;
    auto prev_it = std::prev(it);
    if (std::abs(next_it->first - scan_angle) <
        std::abs(prev_it->first - scan_angle)) {
        return next_it->second;
    } else {
        return prev_it->second;
    }
}

RayId ScanLine3D::get_closest_ray_by_direction(RayId other_ray_id) const {
    // Start with a good first guess for the closest ray based on the scan angle
    Ray3D other_ray = rays->at(other_ray_id);
    RayId closest_ray_id =
        get_closest_ray_by_scan_angle(other_ray.get_scan_angle());

    // Refine the guess by checking the angle between the rays
    RayId best_ray_id = closest_ray_id;
    auto current_angle = rays->at(closest_ray_id).angle_to(other_ray, points);
    double best_angle = current_angle.in_degrees();

    // Check the next rays in the scan line until the angle starts increasing
    RayId next_ray_id = closest_ray_id;
    while (true) {
        auto next_ray_id_new = get_next_ray_id(next_ray_id);
        if (!next_ray_id_new) {
            break;
        }
        next_ray_id = *next_ray_id_new;
        auto next_angle = rays->at(next_ray_id).angle_to(other_ray, points);
        if (next_angle.in_degrees() < best_angle) {
            best_ray_id = next_ray_id;
            best_angle = next_angle.in_degrees();
        } else {
            break;
        }
    }

    // Check the previous rays in the scan line until the angle starts
    // increasing
    RayId prev_ray_id = closest_ray_id;
    while (true) {
        auto prev_ray_id_new = get_prev_ray_id(prev_ray_id);
        if (!prev_ray_id_new) {
            break;
        }
        prev_ray_id = *prev_ray_id_new;
        auto prev_angle = rays->at(prev_ray_id).angle_to(other_ray, points);
        if (prev_angle.in_degrees() < best_angle) {
            best_ray_id = prev_ray_id;
            best_angle = prev_angle.in_degrees();
        } else {
            break;
        }
    }

    return best_ray_id;
}

RayId ScanLine3D::get_closest_ray_by_two_directions(
    const RayId other_ray_id_1, const RayId other_ray_id_2) const {
    // This function is only expected to work well if the two other rays are not
    // too far apart

    // Start with a good first guess for the closest ray based on one of the
    // directions
    RayId closest_ray_id = get_closest_ray_by_direction(other_ray_id_1);
    Ray3D other_ray_1 = rays->at(other_ray_id_1);
    Ray3D other_ray_2 = rays->at(other_ray_id_2);

    // Refine the guess by checking the angle between the rays
    RayId best_ray_id = closest_ray_id;
    auto current_angle_1 =
        rays->at(closest_ray_id).angle_to(other_ray_1, points);
    auto current_angle_2 =
        rays->at(closest_ray_id).angle_to(other_ray_2, points);

    double best_angle =
        current_angle_1.in_degrees() + current_angle_2.in_degrees();

    // Check the next rays in the scan line until the angle starts increasing
    RayId next_ray_id = closest_ray_id;
    while (true) {
        auto next_ray_id_new = get_next_ray_id(next_ray_id);
        if (!next_ray_id_new) {
            break;
        }
        next_ray_id = *next_ray_id_new;
        auto next_angle_1 = rays->at(next_ray_id).angle_to(other_ray_1, points);
        auto next_angle_2 = rays->at(next_ray_id).angle_to(other_ray_2, points);
        double next_angle =
            next_angle_1.in_degrees() + next_angle_2.in_degrees();
        if (next_angle < best_angle) {
            best_ray_id = next_ray_id;
            best_angle = next_angle;
        } else {
            break;
        }
    }

    // Check the previous rays in the scan line until the angle starts
    // increasing
    RayId prev_ray_id = closest_ray_id;
    while (true) {
        auto prev_ray_id_new = get_prev_ray_id(prev_ray_id);
        if (!prev_ray_id_new) {
            break;
        }
        prev_ray_id = *prev_ray_id_new;
        auto prev_angle_1 = rays->at(prev_ray_id).angle_to(other_ray_1, points);
        auto prev_angle_2 = rays->at(prev_ray_id).angle_to(other_ray_2, points);
        double prev_angle =
            prev_angle_1.in_degrees() + prev_angle_2.in_degrees();
        if (prev_angle < best_angle) {
            best_ray_id = prev_ray_id;
            best_angle = prev_angle;
        } else {
            break;
        }
    }

    return best_ray_id;
}

void Topology3D::init(Trajectory trajectory) {
    // This function is called by all constructors to avoid code duplication
    // The implementation is in the .cpp file to avoid including the Trajectory
    // class in the header file

    // Create a mapping from GPS time to indices
    std::cout << "Creating mapping from GPS time to indices..." << std::endl;
    std::map<double, std::vector<PointId>> gps_time_to_indices;
    std::map<double, uint8_t> gps_time_to_scan_direction_flag;
    std::map<double, double> gps_time_to_scan_angle;
    for (PointId i(0); i < points->point_count(); ++i) {
        double gps_time =
            points->get_field_as<double>(pdal::Dimension::Id::GpsTime, i);
        gps_time_to_indices[gps_time].push_back(i);

        uint8_t scan_direction_flag = points->get_field_as<uint8_t>(
            pdal::Dimension::Id::ScanDirectionFlag, i);
        gps_time_to_scan_direction_flag[gps_time] = scan_direction_flag;

        double scan_angle =
            points->get_field_as<double>(pdal::Dimension::Id::ScanAngleRank, i);
        gps_time_to_scan_angle[gps_time] = scan_angle;
    }

    // Create the rays
    std::cout << "Creating rays..." << std::endl;
    for (const auto &entry : gps_time_to_indices) {
        double gps_time = entry.first;
        uint8_t scan_direction_flag = gps_time_to_scan_direction_flag[gps_time];
        const std::vector<PointId> &indices = entry.second;
        Point_3 origin = trajectory.get_point_at_gps_time(gps_time);
        std::vector<int> return_numbers;
        for (PointId idx : indices) {
            int return_number = points->get_field_as<int>(
                pdal::Dimension::Id::ReturnNumber, idx);
            return_numbers.push_back(return_number);
        }
        double scan_angle = gps_time_to_scan_angle[gps_time];
        rays.emplace_back(origin, gps_time, scan_direction_flag, scan_angle,
                          indices, return_numbers);
    }
    std::cout << "Created " << rays.size() << " rays" << std::endl;

    // Create the mapping from GPS time to ray index
    std::cout << "Creating mapping from GPS time to ray ID and from point ID "
                 "to ray ID..."
              << std::endl;
    point_id_to_ray_id.resize(points->point_count());
    for (size_t i = 0; i < rays.size(); ++i) {
        gps_time_to_ray_id[rays[i].get_gps_time()] = RayId(i);
        for (PointId point_id : rays[i].get_point_ids()) {
            point_id_to_ray_id[point_id] = RayId(i);
        }
    }

    // Create an array of ray indices in order of GPS time
    std::cout << "Creating array of ray indices in order of GPS time..."
              << std::endl;
    rays_gps_time_order.resize(rays.size());
    for (size_t i = 0; i < rays.size(); ++i) {
        rays_gps_time_order[i] = RayId(i);
    }
    std::sort(rays_gps_time_order.begin(), rays_gps_time_order.end(),
              [this](RayId a, RayId b) {
                  return this->rays[a].get_gps_time() <
                         this->rays[b].get_gps_time();
              });

    // Create the scan lines
    std::cout << "Creating scan lines..." << std::endl;
    auto shared_rays = std::make_shared<std::vector<Ray3D>>(rays);
    std::vector<RayId> current_scan_line_ray_ids{rays_gps_time_order[0]};
    ray_id_to_scan_line_id.resize(rays.size());
    for (size_t i = 1; i < rays_gps_time_order.size(); ++i) {
        RayId current_ray_id = rays_gps_time_order[i];
        RayId previous_ray_id = rays_gps_time_order[i - 1];
        double current_gps_time = rays[current_ray_id].get_gps_time();
        double previous_gps_time = rays[previous_ray_id].get_gps_time();
        uint8_t current_scan_direction_flag =
            rays[current_ray_id].get_scan_direction_flag();
        uint8_t previous_scan_direction_flag =
            rays[previous_ray_id].get_scan_direction_flag();
        if (std::abs(current_gps_time - previous_gps_time) >=
                SCAN_LINE_MAX_GPS_TIME_DIFFERENCE ||
            current_scan_direction_flag != previous_scan_direction_flag) {
            scan_lines.emplace_back(points, shared_rays,
                                    current_scan_line_ray_ids);
            current_scan_line_ray_ids.clear();
        }
        current_scan_line_ray_ids.push_back(current_ray_id);
        ray_id_to_scan_line_id[current_ray_id] = ScanLineId(scan_lines.size());
    }
    scan_lines.emplace_back(points, shared_rays, current_scan_line_ray_ids);

    // Create mappings from ray index to next and previous ray index in
    // order of GPS time
    std::cout << "Creating mappings from RayId to index in order of GPS time..."
              << std::endl;
    for (size_t i = 0; i < rays_gps_time_order.size(); ++i) {
        ray_id_to_gps_time_order_index[rays_gps_time_order[i]] = i;
    }

    // Create mappings from ray index to next and previous ray index in
    // order of vehicle axis
    std::cout << "Creating mappings from ray index to next and previous ray "
                 "index in order of vehicle axis..."
              << std::endl;
    map_next_ray_vehicle_axis_order.resize(rays.size());
    map_prev_ray_vehicle_axis_order.resize(rays.size());

    // TODO
}

Topology3D::Topology3D(std::vector<pdal::Dimension::Id> predefined_dims,
                       std::vector<ProprietaryDimension> proprietary_dims,
                       pdal::SpatialReference spatial_ref,
                       Trajectory trajectory) {
    // Initialize the point storage
    std::cout << "Initializing point storage..." << std::endl;
    points = std::make_unique<Storage>(predefined_dims, proprietary_dims,
                                       spatial_ref);

    init(trajectory);
}

Topology3D::Topology3D(pdal::PointViewPtr view,
                       pdal::SpatialReference spatial_ref,
                       Trajectory trajectory) {
    // Initialize the point storage
    std::cout << "Initializing point storage..." << std::endl;
    points = std::make_unique<Storage>(view, spatial_ref);

    init(trajectory);
}

Topology3D::Topology3D(StoragePtr storage, Trajectory trajectory) {
    // Initialize the point storage
    std::cout << "Linking point storage..." << std::endl;
    points = storage;

    init(trajectory);
}

// const std::vector<Ray3D> &Topology3D::get_rays() const { return rays; }
// const std::vector<RayId> &Topology3D::get_rays_in_gps_time_order() const {
//     return rays_gps_time_order;
// }
RayId Topology3D::get_ray_id(PointId point_id) const {
    if (point_id < 0 || point_id >= point_id_to_ray_id.size()) {
        throw std::out_of_range("Point ID out of range");
    }
    return point_id_to_ray_id[point_id];
}

const Ray3D &Topology3D::get_ray(RayId i) const {
    if (i < 0 || i >= rays.size()) {
        throw std::out_of_range(std::format(
            "Ray index out of range: {} vs. size: {}", i.get(), rays.size()));
    }
    return rays[i];
}
std::optional<RayId> Topology3D::get_next_ray_in_gps_time_order(RayId i) const {
    if (i < 0 || i >= rays_gps_time_order.size()) {
        throw std::out_of_range(
            std::format("Ray index out of range: {} vs. size: {}", i.get(),
                        rays_gps_time_order.size()));
    }
    auto index = ray_id_to_gps_time_order_index.at(i);
    if (index + 1 < rays_gps_time_order.size()) {
        return rays_gps_time_order[index + 1];
    } else {
        return std::nullopt;
    }
}

ScanLineId Topology3D::get_scan_line_id(RayId ray_id) const {
    if (ray_id < 0 || ray_id >= ray_id_to_scan_line_id.size()) {
        throw std::out_of_range("Ray index out of range");
    }
    return ray_id_to_scan_line_id[ray_id];
}

ScanLineId Topology3D::get_scan_line_id(PointId point_id) const {
    return get_scan_line_id(get_ray_id(point_id));
}

const ScanLine3D &Topology3D::get_scan_line(ScanLineId i) const {
    if (i < 0 || i >= scan_lines.size()) {
        throw std::out_of_range("Scan line index out of range");
    }
    return scan_lines[i];
}

std::optional<RayId> Topology3D::get_prev_ray_in_gps_time_order(RayId i) const {
    if (i < 0 || i >= rays_gps_time_order.size()) {
        throw std::out_of_range("Ray index out of range");
    }
    auto index = ray_id_to_gps_time_order_index.at(i);
    if (index > 0) {
        return rays_gps_time_order[index - 1];
    } else {
        return std::nullopt;
    }
}
// std::optional<RayId> Topology3D::get_next_ray_in_scan_line(RayId i) const {
//     auto potential_next = get_next_ray_in_gps_time_order(i);
//     if (!potential_next) {
//         return std::nullopt;
//     }
//     RayId next = *potential_next;
//     auto current_gps_time = rays[i].get_gps_time();
//     auto next_gps_time = rays[next].get_gps_time();
//     if (std::abs(next_gps_time - current_gps_time) >=
//         SCAN_LINE_MAX_GPS_TIME_DIFFERENCE) {
//         return std::nullopt;
//     } else if (rays[i].get_scan_direction_flag() !=
//                rays[next].get_scan_direction_flag()) {
//         return std::nullopt;
//     } else {
//         return next;
//     }
// }
// std::optional<RayId> Topology3D::get_prev_ray_in_scan_line(RayId i) const {
//     auto potential_prev = get_prev_ray_in_gps_time_order(i);
//     if (!potential_prev) {
//         return std::nullopt;
//     }
//     RayId prev = *potential_prev;
//     auto current_gps_time = rays[i].get_gps_time();
//     auto prev_gps_time = rays[prev].get_gps_time();
//     if (std::abs(prev_gps_time - current_gps_time) >=
//         SCAN_LINE_MAX_GPS_TIME_DIFFERENCE) {
//         return std::nullopt;
//     } else if (rays[i].get_scan_direction_flag() !=
//                rays[prev].get_scan_direction_flag()) {
//         return std::nullopt;
//     } else {
//         return prev;
//     }
// }
std::optional<RayId> Topology3D::get_next_ray_in_vehicle_line(RayId i) const {
    if (i < 0 || i >= map_next_ray_vehicle_axis_order.size()) {
        throw std::out_of_range("Ray index out of range");
    }
    if (i == rays_gps_time_order[rays_gps_time_order.size() - 1]) {
        return std::nullopt;
    }
    return map_next_ray_vehicle_axis_order[i];
}
std::optional<RayId> Topology3D::get_prev_ray_in_vehicle_line(RayId i) const {
    if (i < 0 || i >= map_prev_ray_vehicle_axis_order.size()) {
        throw std::out_of_range("Ray index out of range");
    }
    if (i == rays_gps_time_order[0]) {
        return std::nullopt;
    }
    return map_prev_ray_vehicle_axis_order[i];
}

std::optional<ScanLineId> Topology3D::get_next_scan_line_id(
    std::optional<ScanLineId> scan_line_id) const {
    if (!scan_line_id) {
        return std::nullopt;
    }
    ScanLineId i = *scan_line_id;
    if (i < 0 || i >= scan_lines.size()) {
        throw std::out_of_range("Scan line index out of range");
    }
    if (i == scan_lines.size() - 1) {
        return std::nullopt;
    }
    return ScanLineId(i + 1);
}

std::optional<ScanLineId> Topology3D::get_prev_scan_line_id(
    std::optional<ScanLineId> scan_line_id) const {
    if (!scan_line_id) {
        return std::nullopt;
    }
    ScanLineId i = *scan_line_id;
    if (i < 0 || i >= scan_lines.size()) {
        throw std::out_of_range("Scan line index out of range");
    }
    if (i == 0) {
        return std::nullopt;
    }
    return ScanLineId(i - 1);
}

CustomCGAL::Angle Topology3D::angle_between(RayId ray_1, RayId ray_2) const {
    if (ray_1 < 0 || ray_1 >= rays.size() || ray_2 < 0 ||
        ray_2 >= rays.size()) {
        throw std::out_of_range("Ray index out of range");
    }
    return rays[ray_1].angle_to(rays[ray_2], points);
}