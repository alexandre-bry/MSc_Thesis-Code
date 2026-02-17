#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>

#include <pdal/Dimension.hpp>
#include <pdal/PointView.hpp>
#include <vector>

#include "las.hpp"

const double NEIGHBOUR_MAX_GPS_TIME = 1e-5;
const double NEIGHBOUR_MAX_HORIZONTAL_DISTANCE = 10.0;

// struct PointAttributes {
//     double gps_time;
//     int return_number;
//     int number_of_returns;
//     LASclassification classification;

//     PointAttributes() = default;
//     PointAttributes(double gps_time_, int return_number_,
//                     int number_of_returns_, LASclassification
//                     classification_)
//         : gps_time(gps_time_), return_number(return_number_),
//           number_of_returns(number_of_returns_),
//           classification(classification_) {}
// };

struct Point {
    double x;
    double y;
    double z;

    Point() = default;
    Point(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    Point(pdal::PointViewPtr view, pdal::PointId idx) {
        x = view->getFieldAs<double>(pdal::Dimension::Id::X, idx);
        y = view->getFieldAs<double>(pdal::Dimension::Id::Y, idx);
        z = view->getFieldAs<double>(pdal::Dimension::Id::Z, idx);
    }

    const Point operator+(const Point &other) const {
        return Point(x + other.x, y + other.y, z + other.z);
    }

    const Point operator-(const Point &other) const {
        return Point(x - other.x, y - other.y, z - other.z);
    }

    const Point operator*(double scalar) const {
        return Point(x * scalar, y * scalar, z * scalar);
    }

    const Point operator/(double scalar) const {
        return Point(x / scalar, y / scalar, z / scalar);
    }

    double distance_to(const Point &other) const {
        return sqrt((x - other.x) * (x - other.x) +
                    (y - other.y) * (y - other.y) +
                    (z - other.z) * (z - other.z));
    }

    double horizontal_distance_to(const Point &other) const {
        return sqrt((x - other.x) * (x - other.x) +
                    (y - other.y) * (y - other.y));
    }

    double vertical_distance_to(const Point &other) const {
        return std::abs(z - other.z);
    }

    double signed_vertical_distance_to(const Point &other) const {
        return other.z - z;
    }

    bool is_neighbour_in_distance(const Point &other) const {
        return horizontal_distance_to(other) <
               NEIGHBOUR_MAX_HORIZONTAL_DISTANCE;
    }
};

struct PointWithAttributes : Point {
    double gps_time;
    int return_number;
    int number_of_returns;
    LASclassification classification;
    int point_source_id;

    PointWithAttributes() = default;
    PointWithAttributes(double x_, double y_, double z_, double gps_time_,
                        int return_number_, int number_of_returns_,
                        LASclassification classification_, int point_source_id_)
        : Point(x_, y_, z_), gps_time(gps_time_), return_number(return_number_),
          number_of_returns(number_of_returns_),
          classification(classification_), point_source_id(point_source_id_) {}
    PointWithAttributes(pdal::PointViewPtr view, pdal::PointId idx) {
        x = view->getFieldAs<double>(pdal::Dimension::Id::X, idx);
        y = view->getFieldAs<double>(pdal::Dimension::Id::Y, idx);
        z = view->getFieldAs<double>(pdal::Dimension::Id::Z, idx);
        gps_time = view->getFieldAs<double>(pdal::Dimension::Id::GpsTime, idx);
        return_number =
            view->getFieldAs<int>(pdal::Dimension::Id::ReturnNumber, idx);
        number_of_returns =
            view->getFieldAs<int>(pdal::Dimension::Id::NumberOfReturns, idx);
        point_source_id =
            view->getFieldAs<int>(pdal::Dimension::Id::PointSourceId, idx);
        classification =
            static_cast<LASclassification>(view->getFieldAs<uint8_t>(
                pdal::Dimension::Id::Classification, idx));
    }
};

struct PointsWithAttributes {
    std::vector<PointWithAttributes> points;
    std::map<double, std::vector<std::size_t>> gps_time_to_in_indices;
    std::vector<std::size_t> indices_sorted_to_in; // Mapping from sorted
                                                   // indices to input indices
    std::vector<std::size_t> indices_in_to_sorted; // Mapping from input indices
                                                   // to sorted indices

    PointsWithAttributes() = default;
    PointsWithAttributes(pdal::PointViewPtr view) {
        // Initialize the points vector
        pdal::PointId num_points = view->size();
        points.reserve(num_points);
        for (pdal::PointId i = 0; i < num_points; ++i) {
            points.emplace_back(view, i);
        }

        indices_sorted_to_in.reserve(num_points);
        for (std::size_t i = 0; i < num_points; ++i) {
            indices_sorted_to_in.emplace_back(i);
        }

        // Sort the points by GPS time
        std::sort(indices_sorted_to_in.begin(), indices_sorted_to_in.end(),
                  [this](std::size_t a, std::size_t b) {
                      auto gps_time_a = points[a].gps_time;
                      auto gps_time_b = points[b].gps_time;
                      if (gps_time_a == gps_time_b) {
                          auto return_number_a = points[a].return_number;
                          auto return_number_b = points[b].return_number;
                          return return_number_a < return_number_b;
                      } else {
                          return points[a].gps_time < points[b].gps_time;
                      }
                  });

        // Create the mapping from input indices to sorted indices
        indices_in_to_sorted.resize(num_points);
        for (std::size_t sorted_idx = 0;
             sorted_idx < indices_sorted_to_in.size(); ++sorted_idx) {
            std::size_t in_idx = indices_sorted_to_in[sorted_idx];
            indices_in_to_sorted[in_idx] = sorted_idx;
        }

        // Create the mapping from GPS time to indices
        for (auto i : indices_sorted_to_in) {
            gps_time_to_in_indices[points[i].gps_time].push_back(i);
        }
    }

    bool are_neighbours(std::size_t index_1, std::size_t index_2) const {
        const PointWithAttributes &p1 = points[index_1];
        const PointWithAttributes &p2 = points[index_2];
        double gps_time1 = p1.gps_time;
        double gps_time2 = p2.gps_time;
        double horizontal_distance = p1.horizontal_distance_to(p2);
        return std::abs(gps_time1 - gps_time2) < NEIGHBOUR_MAX_GPS_TIME &&
               horizontal_distance < NEIGHBOUR_MAX_HORIZONTAL_DISTANCE;
    }

    bool any_neighbours(std::size_t index,
                        const std::vector<std::size_t> &indices) const {
        for (auto other_index : indices) {
            if (are_neighbours(index, other_index)) {
                return true;
            }
        }
        return false;
    }

    bool any_neighbours(const std::vector<std::size_t> &indices_1,
                        const std::vector<std::size_t> &indices_2) const {
        for (std::size_t index_1 : indices_1) {
            for (std::size_t index_2 : indices_2) {
                if (are_neighbours(index_1, index_2)) {
                    return true;
                }
            }
        }
        return false;
    }

    std::optional<double> get_prev_gps_time(double gps_time) const {
        auto it = gps_time_to_in_indices.lower_bound(gps_time);
        if (it != gps_time_to_in_indices.begin()) {
            --it;
            return it->first;
        } else {
            return std::nullopt;
        }
    }

    std::optional<double> get_next_gps_time(double gps_time) const {
        auto it = gps_time_to_in_indices.upper_bound(gps_time);
        if (it != gps_time_to_in_indices.end()) {
            return it->first;
        } else {
            return std::nullopt;
        }
    }

    const std::vector<std::size_t> &
    get_indices_for_gps_time(double gps_time) const {
        return gps_time_to_in_indices.at(gps_time);
    }

    const std::vector<std::size_t> &
    get_indices_for_prev_gps_time(double gps_time) const {
        auto prev_gps_time = get_prev_gps_time(gps_time);
        if (prev_gps_time) {
            return gps_time_to_in_indices.at(*prev_gps_time);
        } else {
            static const std::vector<std::size_t> empty;
            return empty;
        }
    }

    const std::vector<std::size_t> &
    get_indices_for_next_gps_time(double gps_time) const {
        auto next_gps_time = get_next_gps_time(gps_time);
        if (next_gps_time) {
            return gps_time_to_in_indices.at(*next_gps_time);
        } else {
            static const std::vector<std::size_t> empty;
            return empty;
        }
    }

    const std::size_t get_index_of_highest_return(double gps_time) const {
        const auto &indices = get_indices_for_gps_time(gps_time);
        std::size_t index_of_highest_return = indices[0];
        for (std::size_t index : indices) {
            if (points[index].return_number >
                points[index_of_highest_return].return_number) {
                index_of_highest_return = index;
            }
        }
        return index_of_highest_return;
    }

    const std::size_t get_index_of_lowest_return(double gps_time) const {
        const auto &indices = get_indices_for_gps_time(gps_time);
        std::size_t index_of_lowest_return = indices[0];
        for (std::size_t index : indices) {
            if (points[index].return_number <
                points[index_of_lowest_return].return_number) {
                index_of_lowest_return = index;
            }
        }
        return index_of_lowest_return;
    }

    /**
     * @brief Returns an iterator over groups of points with the same GPS
     * time, in order of GPS time.
     */
    std::vector<std::vector<std::size_t>> get_groups_in_gps_time_order() const {
        std::vector<std::vector<std::size_t>> groups;
        for (const auto &entry : gps_time_to_in_indices) {
            groups.push_back(entry.second);
        }
        return groups;
    }

    std::vector<const PointWithAttributes *>
    get_points_by_in_indices(const std::vector<std::size_t> &indices) const {
        std::vector<const PointWithAttributes *> result;
        result.reserve(indices.size());
        for (std::size_t index : indices) {
            result.push_back(&points.at(index));
        }
        return result;
    }

    /**
     * @brief Operator [] to access points by initial index.
     *
     */
    const PointWithAttributes &operator[](std::size_t index) const {
        return points[index];
    }

    std::size_t size() const { return points.size(); }

    /**
     * @brief Converts an input index to a GPS time sorted index.
     *
     * @param in_index Input index
     * @return std::size_t GPS time sorted index
     */
    std::size_t to_sorted_index(std::size_t initial_index) const {
        if (initial_index >= indices_in_to_sorted.size()) {
            throw std::out_of_range("Input index out of range");
        }
        return indices_in_to_sorted[initial_index];
    }

    std::size_t to_initial_index(std::size_t sorted_index) const {
        if (sorted_index >= indices_sorted_to_in.size()) {
            throw std::out_of_range("Sorted index out of range");
        }
        return indices_sorted_to_in[sorted_index];
    }
};
