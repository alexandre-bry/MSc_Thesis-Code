#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>

#include <pdal/Dimension.hpp>
#include <pdal/PointView.hpp>
#include <string>
#include <vector>

const double NEIGHBOUR_MAX_GPS_TIME = 1e-5;
const double NEIGHBOUR_MAX_HORIZONTAL_DISTANCE = 10.0;

namespace LASclassification {
enum class Value : uint8_t {
    Unclassified = 0,
    Unassigned = 1,
    Ground = 2,
    LowVegetation = 3,
    MediumVegetation = 4,
    HighVegetation = 5,
    Building = 6,
    LowPoint = 7,
    ModelKeyPoint = 8,
    Water = 9,
    Rail = 10,
    RoadSurface = 11,
    Overlap = 12,
    WireGuard = 13,
    WireConductor = 14,
    TransmissionTower = 15,
    WireConnector = 16,
    BridgeDeck = 17,
    HighNoise = 18,
    PermanentOverground = 64,    // Specific to LiDAR HD
    VirtualPoints = 66,          // Specific to LiDAR HD
    MiscellaneousBuildings = 67, // Specific to LiDAR HD
};
inline std::string name(Value value) {
    switch (value) {
    case Value::Unclassified:
        return "Unclassified";
    case Value::Unassigned:
        return "Unassigned";
    case Value::Ground:
        return "Ground";
    case Value::LowVegetation:
        return "Low Vegetation";
    case Value::MediumVegetation:
        return "Medium Vegetation";
    case Value::HighVegetation:
        return "High Vegetation";
    case Value::Building:
        return "Building";
    case Value::LowPoint:
        return "Low Point";
    case Value::ModelKeyPoint:
        return "Model Key Point";
    case Value::Water:
        return "Water";
    case Value::Rail:
        return "Rail";
    case Value::RoadSurface:
        return "Road Surface";
    case Value::Overlap:
        return "Overlap";
    case Value::WireGuard:
        return "Wire Guard";
    case Value::WireConductor:
        return "Wire Conductor";
    case Value::TransmissionTower:
        return "Transmission Tower";
    case Value::WireConnector:
        return "Wire Connector";
    case Value::BridgeDeck:
        return "Bridge Deck";
    case Value::HighNoise:
        return "High Noise";
    case Value::PermanentOverground:
        return "Permanent Overground (LiDAR HD)";
    case Value::VirtualPoints:
        return "Virtual Points (LiDAR HD)";
    case Value::MiscellaneousBuildings:
        return "Miscellaneous Buildings (LiDAR HD)";
    default:
        throw std::runtime_error("Unknown LAS classification value: " +
                                 std::to_string(static_cast<uint8_t>(value)));
    }
}
} // namespace LASclassification

namespace CustomDimensions {
enum class Id {
    ReturnNumberComputed,
    NumberOfReturnsComputed,
    DownSignedVertGap,
    UpSignedVertGap,
    IsRoofEdge,
    IsFootEdge,
    IsGenerated,
};
inline std::string name(Id id) {
    switch (id) {
    case Id::ReturnNumberComputed:
        return "ReturnNumberComputed";
    case Id::NumberOfReturnsComputed:
        return "NumberOfReturnsComputed";
    case Id::DownSignedVertGap:
        return "DownSignedVertGap";
    case Id::UpSignedVertGap:
        return "UpSignedVertGap";
    case Id::IsRoofEdge:
        return "IsRoofEdge";
    case Id::IsFootEdge:
        return "IsFootEdge";
    case Id::IsGenerated:
        return "IsGenerated";
    default:
        throw std::runtime_error("Unknown custom dimension ID");
    }
}

inline pdal::Dimension::Type type(Id id) {
    switch (id) {
    case Id::ReturnNumberComputed:
        return pdal::Dimension::Type::Unsigned8;
    case Id::NumberOfReturnsComputed:
        return pdal::Dimension::Type::Unsigned8;
    case Id::DownSignedVertGap:
        return pdal::Dimension::Type::Double;
    case Id::UpSignedVertGap:
        return pdal::Dimension::Type::Double;
    case Id::IsRoofEdge:
        return pdal::Dimension::Type::Unsigned8;
    case Id::IsFootEdge:
        return pdal::Dimension::Type::Unsigned8;
    case Id::IsGenerated:
        return pdal::Dimension::Type::Unsigned8;
    default:
        throw std::runtime_error("Unknown custom dimension ID");
    }
}

} // namespace CustomDimensions

struct ProprietaryDimension {
    std::string name;
    pdal::Dimension::Type type;

    ProprietaryDimension(const std::string &n, pdal::Dimension::Type t)
        : name(n), type(t) {}

    ProprietaryDimension(const CustomDimensions::Id id)
        : name(CustomDimensions::name(id)), type(CustomDimensions::type(id)) {}
};

struct Point2D {
    double x;
    double y;

    Point2D() = default;
    Point2D(double x_, double y_) : x(x_), y(y_) {}
};

struct Point3D {
    double x;
    double y;
    double z;

    Point3D() = default;
    Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    Point3D(pdal::PointViewPtr view, pdal::PointId idx) {
        x = view->getFieldAs<double>(pdal::Dimension::Id::X, idx);
        y = view->getFieldAs<double>(pdal::Dimension::Id::Y, idx);
        z = view->getFieldAs<double>(pdal::Dimension::Id::Z, idx);
    }

    const Point3D operator+(const Point3D &other) const {
        return Point3D(x + other.x, y + other.y, z + other.z);
    }

    const Point3D operator-(const Point3D &other) const {
        return Point3D(x - other.x, y - other.y, z - other.z);
    }

    const Point3D operator*(double scalar) const {
        return Point3D(x * scalar, y * scalar, z * scalar);
    }

    const Point3D operator/(double scalar) const {
        return Point3D(x / scalar, y / scalar, z / scalar);
    }

    double distance_to(const Point3D &other) const {
        return sqrt((x - other.x) * (x - other.x) +
                    (y - other.y) * (y - other.y) +
                    (z - other.z) * (z - other.z));
    }

    double horizontal_distance_to(const Point3D &other) const {
        return sqrt((x - other.x) * (x - other.x) +
                    (y - other.y) * (y - other.y));
    }

    double vertical_distance_to(const Point3D &other) const {
        return std::abs(z - other.z);
    }

    double signed_vertical_distance_to(const Point3D &other) const {
        return other.z - z;
    }

    bool is_neighbour_in_distance(const Point3D &other) const {
        return horizontal_distance_to(other) <
               NEIGHBOUR_MAX_HORIZONTAL_DISTANCE;
    }
};

struct Point3DWithAttributes : Point3D {
  private:
    std::optional<int> return_number_computed;
    std::optional<int> number_of_returns_computed;

  public:
    double gps_time;
    int return_number_attribute;
    int number_of_returns_attribute;
    LASclassification::Value classification;
    int point_source_id;

    Point3DWithAttributes() = default;
    // Point3DWithAttributes(double x_, double y_, double z_, double gps_time_,
    //                       int return_number_, int number_of_returns_,
    //                       LASclassification::Value classification_,
    //                       int point_source_id_)
    // : Point3D(x_, y_, z_), gps_time(gps_time_),
    //   return_number(return_number_), number_of_returns(number_of_returns_),
    //   classification(classification_), point_source_id(point_source_id_) {}
    Point3DWithAttributes(pdal::PointViewPtr view, pdal::PointId idx) {
        x = view->getFieldAs<double>(pdal::Dimension::Id::X, idx);
        y = view->getFieldAs<double>(pdal::Dimension::Id::Y, idx);
        z = view->getFieldAs<double>(pdal::Dimension::Id::Z, idx);
        gps_time = view->getFieldAs<double>(pdal::Dimension::Id::GpsTime, idx);
        return_number_attribute =
            view->getFieldAs<int>(pdal::Dimension::Id::ReturnNumber, idx);
        number_of_returns_attribute =
            view->getFieldAs<int>(pdal::Dimension::Id::NumberOfReturns, idx);
        point_source_id =
            view->getFieldAs<int>(pdal::Dimension::Id::PointSourceId, idx);
        classification =
            static_cast<LASclassification::Value>(view->getFieldAs<uint8_t>(
                pdal::Dimension::Id::Classification, idx));
    }

    void set_return_number_computed(int return_number) {
        return_number_computed = return_number;
    }

    void set_number_of_returns_computed(int number_of_returns) {
        number_of_returns_computed = number_of_returns;
    }

    int get_return_number_computed() const {
        if (!return_number_computed) {
            throw std::runtime_error(
                "Return number computed is not set for this point");
        }
        return *return_number_computed;
    }

    int get_number_of_returns_computed() const {
        if (!number_of_returns_computed) {
            throw std::runtime_error(
                "Number of returns computed is not set for this point");
        }
        return *number_of_returns_computed;
    }
};

struct Points3DWithAttributes {
    std::vector<Point3DWithAttributes> points;
    // Additional data structures for efficient queries can be added here

    Points3DWithAttributes() = default;
    Points3DWithAttributes(pdal::PointViewPtr view) {
        pdal::PointId num_points = view->size();
        points.reserve(num_points);
        for (pdal::PointId i = 0; i < num_points; ++i) {
            points.emplace_back(view, i);
        }
    }

    std::size_t size() const { return points.size(); }

    /**
     * @brief Operator [] to access points by initial index.
     *
     */
    const Point3DWithAttributes &operator[](std::size_t index) const {
        return points[index];
    }
};

struct Points3DAttrGPSSorted : Points3DWithAttributes {
    std::map<double, std::vector<std::size_t>> gps_time_to_in_indices;
    std::vector<std::size_t> indices_sorted_to_in; // Mapping from sorted
                                                   // indices to input indices
    std::vector<std::size_t> indices_in_to_sorted; // Mapping from input indices
                                                   // to sorted indices

    Points3DAttrGPSSorted() = default;
    Points3DAttrGPSSorted(pdal::PointViewPtr view) {
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
        std::cout << "Sorting points by GPS time..." << std::endl;
        std::sort(indices_sorted_to_in.begin(), indices_sorted_to_in.end(),
                  [this](std::size_t a, std::size_t b) {
                      auto gps_time_a = points[a].gps_time;
                      auto gps_time_b = points[b].gps_time;
                      //   if (gps_time_a == gps_time_b) {
                      //       auto return_number_a =
                      //           points[a].return_number_attribute;
                      //       auto return_number_b =
                      //           points[b].return_number_attribute;
                      //       return return_number_a < return_number_b;
                      //   } else {
                      //       return gps_time_a < gps_time_b;
                      //   }
                      return gps_time_a < gps_time_b;
                  });

        // Create the mapping from input indices to sorted indices
        std::cout << "Creating mapping from input indices to sorted indices..."
                  << std::endl;
        indices_in_to_sorted.resize(num_points);
        for (std::size_t sorted_idx = 0;
             sorted_idx < indices_sorted_to_in.size(); ++sorted_idx) {
            std::size_t in_idx = indices_sorted_to_in[sorted_idx];
            indices_in_to_sorted[in_idx] = sorted_idx;
        }

        // Create the mapping from GPS time to indices
        std::cout << "Creating mapping from GPS time to indices..."
                  << std::endl;
        for (auto i : indices_sorted_to_in) {
            gps_time_to_in_indices[points[i].gps_time].push_back(i);
        }

        // Compute the number of returns and return number for each group of
        // points with the same GPS time
        std::cout << "Computing return numbers and number of returns for each "
                     "group of points with the same GPS time..."
                  << std::endl;
        for (const auto &entry : gps_time_to_in_indices) {
            auto indices = entry.second;
            int number_of_returns = indices.size();
            std::sort(indices.begin(), indices.end(),
                      [this](std::size_t a, std::size_t b) {
                          return points[a].return_number_attribute <
                                 points[b].return_number_attribute;
                      });
            for (uint8_t i = 0; i < indices.size(); ++i) {
                std::size_t index = indices[i];
                points[index].set_number_of_returns_computed(number_of_returns);
                points[index].set_return_number_computed(i + 1);
            }
        }

        std::cout << "Finished initializing Points3DWithAttributes."
                  << std::endl;
    }

    bool are_neighbours(std::size_t index_1, std::size_t index_2) const {
        const Point3DWithAttributes &p1 = points[index_1];
        const Point3DWithAttributes &p2 = points[index_2];
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
            if (points[index].get_return_number_computed() >
                points[index_of_highest_return].get_return_number_computed()) {
                index_of_highest_return = index;
            }
        }
        return index_of_highest_return;
    }

    const std::size_t get_index_of_lowest_return(double gps_time) const {
        const auto &indices = get_indices_for_gps_time(gps_time);
        std::size_t index_of_lowest_return = indices[0];
        for (std::size_t index : indices) {
            if (points[index].get_return_number_computed() <
                points[index_of_lowest_return].get_return_number_computed()) {
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

    std::vector<const Point3DWithAttributes *>
    get_points_by_in_indices(const std::vector<std::size_t> &indices) const {
        std::vector<const Point3DWithAttributes *> result;
        result.reserve(indices.size());
        for (std::size_t index : indices) {
            result.push_back(&points.at(index));
        }
        return result;
    }

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
