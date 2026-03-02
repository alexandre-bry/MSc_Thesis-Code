#include "points.hpp"
#include <cstdint>
#include <optional>

PointStorage::PointStorage(std::vector<pdal::Dimension::Id> predefined_dims,
                           std::vector<ProprietaryDimension> proprietary_dims,
                           pdal::SpatialReference spatial_ref) {

    for (auto dim : predefined_dims) {
        table.layout()->registerDim(dim);
    }
    for (auto dim : proprietary_dims) {
        table.layout()->registerOrAssignDim(dim.name, dim.type);
    }

    table.clearSpatialReferences();
    table.setSpatialReference(spatial_ref);

    view = pdal::PointViewPtr(new pdal::PointView(table));
}

PointStorage::PointStorage(pdal::PointViewPtr view,
                           pdal::SpatialReference spatial_ref) {
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

    PointStorage(predefined_dims, proprietary_dims, spatial_ref);
}

std::size_t PointStorage::pointCount() { return view->size(); }

Points3DRay::Points3DRay(const Point_3 &origin_, double gps_time_,
                         uint8_t scan_direction_flag_,
                         const std::vector<pdal::PointId> &point_ids_,
                         const std::vector<int> &return_numbers)
    : origin(origin_), gps_time(gps_time_),
      scan_direction_flag(scan_direction_flag_) {
    // Sort the point IDs by return number
    std::vector<std::pair<pdal::PointId, int>> point_id_and_return_number;
    for (size_t i = 0; i < point_ids_.size(); ++i) {
        pdal::PointId point_id = point_ids_[i];
        int return_number = return_numbers[i];
        point_id_and_return_number.push_back(
            std::make_pair(point_id, return_number));
    }

    std::sort(point_id_and_return_number.begin(),
              point_id_and_return_number.end(),
              [](const std::pair<pdal::PointId, int> &a,
                 const std::pair<pdal::PointId, int> &b) {
                  return a.second < b.second;
              });

    for (const auto &pair : point_id_and_return_number) {
        point_ids.push_back(pair.first);
    }
}

pdal::PointId Points3DRay::get_in_return_order(int return_number) const {
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
    return point_ids.at(return_number);
}

Points3DStructured::Points3DStructured(
    std::vector<pdal::Dimension::Id> predefined_dims,
    std::vector<ProprietaryDimension> proprietary_dims,
    pdal::SpatialReference spatial_ref, Trajectory trajectory) {
    // Initialize the point storage
    std::cout << "Initializing point storage..." << std::endl;
    points = std::make_unique<PointStorage>(predefined_dims, proprietary_dims,
                                            spatial_ref);

    // Create a mapping from GPS time to indices
    std::cout << "Creating mapping from GPS time to indices..." << std::endl;
    std::map<double, std::vector<pdal::PointId>> gps_time_to_indices;
    std::map<double, uint8_t> gps_time_to_scan_direction_flag;
    for (pdal::PointId i = 0; i < points->pointCount(); ++i) {
        double gps_time =
            points->getFieldAs<double>(pdal::Dimension::Id::GpsTime, i);
        gps_time_to_indices[gps_time].push_back(i);

        uint8_t scan_direction_flag = points->getFieldAs<uint8_t>(
            pdal::Dimension::Id::ScanDirectionFlag, i);
        gps_time_to_scan_direction_flag[gps_time] = scan_direction_flag;
    }

    // Create the rays
    std::cout << "Creating rays..." << std::endl;
    for (const auto &entry : gps_time_to_indices) {
        double gps_time = entry.first;
        uint8_t scan_direction_flag = gps_time_to_scan_direction_flag[gps_time];
        const std::vector<pdal::PointId> &indices = entry.second;
        Point_3 origin = trajectory.get_point_at_gps_time(gps_time);
        std::vector<int> return_numbers;
        for (pdal::PointId idx : indices) {
            int return_number =
                points->getFieldAs<int>(pdal::Dimension::Id::ReturnNumber, idx);
            return_numbers.push_back(return_number);
        }
        rays.emplace_back(origin, gps_time, scan_direction_flag, indices,
                          return_numbers);
    }

    // Create the mapping from GPS time to ray index
    std::cout << "Creating mapping from GPS time to ray index..." << std::endl;
    for (size_t i = 0; i < rays.size(); ++i) {
        gps_time_to_ray_index[rays[i].get_gps_time()] = i;
    }

    // Create an array of ray indices in order of GPS time
    std::cout << "Creating array of ray indices in order of GPS time..."
              << std::endl;
    gps_time_order.resize(rays.size());
    for (size_t i = 0; i < rays.size(); ++i) {
        gps_time_order[i] = i;
    }
    std::sort(gps_time_order.begin(), gps_time_order.end(),
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
            map_prev_ray_gps_time_order[i] = gps_time_order[i - 1];
        } else {
            map_prev_ray_gps_time_order[i] = -1;
        }
        if (i < rays.size() - 1) {
            map_next_ray_gps_time_order[i] = gps_time_order[i + 1];
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

const std::vector<Points3DRay> &Points3DStructured::get_rays() const {
    return rays;
}
const Points3DRay &Points3DStructured::get_ray(RayId i) const {
    return rays[i];
}
std::optional<RayId>
Points3DStructured::get_next_ray_in_gps_time_order(RayId i) const {
    if (i < 0 || i >= map_next_ray_gps_time_order.size()) {
        throw std::out_of_range("Ray index out of range");
    }
    if (i == gps_time_order[gps_time_order.size() - 1]) {
        return std::nullopt;
    }
    return map_next_ray_gps_time_order[i];
}
std::optional<RayId>
Points3DStructured::get_prev_ray_in_gps_time_order(RayId i) const {
    if (i < 0 || i >= map_prev_ray_gps_time_order.size()) {
        throw std::out_of_range("Ray index out of range");
    }
    if (i == gps_time_order[0]) {
        return std::nullopt;
    }
    return map_prev_ray_gps_time_order[i];
}
std::optional<RayId>
Points3DStructured::get_next_ray_in_scan_line(RayId i) const {
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
std::optional<RayId>
Points3DStructured::get_prev_ray_in_scan_line(RayId i) const {
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
std::optional<RayId>
Points3DStructured::get_next_ray_in_vehicle_line(RayId i) const {
    if (i < 0 || i >= map_next_ray_vehicle_axis_order.size()) {
        throw std::out_of_range("Ray index out of range");
    }
    if (i == gps_time_order[gps_time_order.size() - 1]) {
        return std::nullopt;
    }
    return map_next_ray_vehicle_axis_order[i];
}
std::optional<RayId>
Points3DStructured::get_prev_ray_in_vehicle_line(RayId i) const {
    if (i < 0 || i >= map_prev_ray_vehicle_axis_order.size()) {
        throw std::out_of_range("Ray index out of range");
    }
    if (i == gps_time_order[0]) {
        return std::nullopt;
    }
    return map_prev_ray_vehicle_axis_order[i];
}