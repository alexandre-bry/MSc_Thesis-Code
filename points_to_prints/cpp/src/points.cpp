#include "points.hpp"

#include <cstdint>
#include <optional>

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

Ray3D::Ray3D(const Point_3 &origin_, double gps_time_,
             uint8_t scan_direction_flag_,
             const std::vector<PointId> &point_ids_,
             const std::vector<int> &return_numbers)
    : origin(origin_), gps_time(gps_time_),
      scan_direction_flag(scan_direction_flag_) {
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

void Topology3D::init(Trajectory trajectory) {
    // This function is called by both constructors to avoid code duplication
    // The implementation is in the .cpp file to avoid including the Trajectory
    // class in the header file

    // Create a mapping from GPS time to indices
    std::cout << "Creating mapping from GPS time to indices..." << std::endl;
    std::map<double, std::vector<PointId>> gps_time_to_indices;
    std::map<double, uint8_t> gps_time_to_scan_direction_flag;
    for (PointId i = 0; i < points->point_count(); ++i) {
        double gps_time =
            points->get_field_as<double>(pdal::Dimension::Id::GpsTime, i);
        gps_time_to_indices[gps_time].push_back(i);

        uint8_t scan_direction_flag = points->get_field_as<uint8_t>(
            pdal::Dimension::Id::ScanDirectionFlag, i);
        gps_time_to_scan_direction_flag[gps_time] = scan_direction_flag;
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
        rays.emplace_back(origin, gps_time, scan_direction_flag, indices,
                          return_numbers);
    }

    // Create the mapping from GPS time to ray index
    std::cout << "Creating mapping from GPS time to ray ID and from point ID "
                 "to ray ID..."
              << std::endl;
    point_id_to_ray_id.resize(points->point_count());
    for (size_t i = 0; i < rays.size(); ++i) {
        gps_time_to_ray_id[rays[i].get_gps_time()] = i;
        for (PointId point_id : rays[i].get_point_ids()) {
            point_id_to_ray_id[point_id] = i;
        }
    }

    // Create an array of ray indices in order of GPS time
    std::cout << "Creating array of ray indices in order of GPS time..."
              << std::endl;
    rays_gps_time_order.resize(rays.size());
    for (size_t i = 0; i < rays.size(); ++i) {
        rays_gps_time_order[i] = i;
    }
    std::sort(rays_gps_time_order.begin(), rays_gps_time_order.end(),
              [this](size_t a, size_t b) {
                  return rays[a].get_gps_time() < rays[b].get_gps_time();
              });

    // Create mappings from ray index to next and previous ray index in
    // order of GPS time
    std::cout << "Creating mappings from ray index to next and previous ray "
                 "index in order of GPS time..."
              << std::endl;
    map_next_ray_gps_time_order.resize(rays.size());
    map_prev_ray_gps_time_order.resize(rays.size());
    for (RayId i = 0; i < rays.size(); ++i) {
        if (i > 0) {
            map_prev_ray_gps_time_order[i] = rays_gps_time_order[i - 1];
        } else {
            map_prev_ray_gps_time_order[i] = -1;
        }
        if (i < rays.size() - 1) {
            map_next_ray_gps_time_order[i] = rays_gps_time_order[i + 1];
        } else {
            map_next_ray_gps_time_order[i] = -1;
        }
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
RayId Topology3D::get_ray_id_from_point_id(PointId point_id) const {
    if (point_id < 0 || point_id >= point_id_to_ray_id.size()) {
        throw std::out_of_range("Point ID out of range");
    }
    return point_id_to_ray_id[point_id];
}
const Ray3D &Topology3D::get_ray(RayId i) const { return rays[i]; }
std::optional<RayId> Topology3D::get_next_ray_in_gps_time_order(RayId i) const {
    if (i < 0 || i >= map_next_ray_gps_time_order.size()) {
        throw std::out_of_range("Ray index out of range");
    }
    if (i == rays_gps_time_order[rays_gps_time_order.size() - 1]) {
        return std::nullopt;
    }
    return map_next_ray_gps_time_order[i];
}
std::optional<RayId> Topology3D::get_prev_ray_in_gps_time_order(RayId i) const {
    if (i < 0 || i >= map_prev_ray_gps_time_order.size()) {
        throw std::out_of_range("Ray index out of range");
    }
    if (i == rays_gps_time_order[0]) {
        return std::nullopt;
    }
    return map_prev_ray_gps_time_order[i];
}
std::optional<RayId> Topology3D::get_next_ray_in_scan_line(RayId i) const {
    auto potential_next = get_next_ray_in_gps_time_order(i);
    if (!potential_next) {
        return std::nullopt;
    }
    RayId next = *potential_next;
    auto current_gps_time = rays[i].get_gps_time();
    auto next_gps_time = rays[next].get_gps_time();
    if (std::abs(next_gps_time - current_gps_time) >=
        SCAN_LINE_MAX_GPS_TIME_DIFFERENCE) {
        return std::nullopt;
    } else if (rays[i].get_scan_direction_flag() !=
               rays[next].get_scan_direction_flag()) {
        return std::nullopt;
    } else {
        return next;
    }
}
std::optional<RayId> Topology3D::get_prev_ray_in_scan_line(RayId i) const {
    auto potential_prev = get_prev_ray_in_gps_time_order(i);
    if (!potential_prev) {
        return std::nullopt;
    }
    RayId prev = *potential_prev;
    auto current_gps_time = rays[i].get_gps_time();
    auto prev_gps_time = rays[prev].get_gps_time();
    if (std::abs(prev_gps_time - current_gps_time) >=
        SCAN_LINE_MAX_GPS_TIME_DIFFERENCE) {
        return std::nullopt;
    } else if (rays[i].get_scan_direction_flag() !=
               rays[prev].get_scan_direction_flag()) {
        return std::nullopt;
    } else {
        return prev;
    }
}
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