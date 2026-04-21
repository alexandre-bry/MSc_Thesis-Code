#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <pdal/DimUtil.hpp>
#include <pdal/pdal_types.hpp>
#include <vector>

#include <pdal/Dimension.hpp>
#include <pdal/PointView.hpp>

#include "geometry.hpp"
#include "las/enums.hpp"
#include "las/trajectory.hpp"
#include "utils/cgal.hpp"
#include "utils/kd_tree.hpp"
#include "utils/strong_types.hpp"

const double NEIGHBOUR_MAX_GPS_TIME = 1e-5;
const double NEIGHBOUR_MAX_HORIZONTAL_DISTANCE = 10.0;

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
    Points3DWithAttributes(const std::vector<Point3DWithAttributes> &pts)
        : points(pts) {}
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

namespace PtsStructs {

struct PointIdTag {};
struct RayIdTag {};
struct ScanLineIdTag {};

typedef StrongType<PointIdTag, pdal::PointId> PointId;
typedef StrongType<RayIdTag, size_t> RayId;
typedef StrongType<ScanLineIdTag, size_t> ScanLineId;

struct Storage {
  protected:
    std::vector<pdal::Dimension::Id> predefined_dims;
    std::vector<CustomDimensions::Id> custom_dims;
    pdal::PointViewPtr view;
    std::shared_ptr<pdal::PointTable> table;
    std::shared_ptr<KdTree_2> las_kd_tree_2;
    std::shared_ptr<KdTree_3> las_kd_tree_3;
    std::vector<Point_3> cached_points;

  private:
    void init(std::vector<pdal::Dimension::Id> predefined_dims,
              std::vector<ProprietaryDimension> proprietary_dims,
              pdal::SpatialReference spatial_ref);

  public:
    Storage() = default;
    Storage(std::vector<pdal::Dimension::Id> predefined_dims,
            std::vector<ProprietaryDimension> proprietary_dims,
            pdal::SpatialReference spatial_ref);
    Storage(pdal::PointViewPtr view, pdal::SpatialReference spatial_ref);
    Storage(pdal::PointViewPtr view, std::shared_ptr<pdal::PointTable> table);

    // Delete copy and move - Storage is non-transferrable
    Storage(const Storage &) = delete;
    Storage &operator=(const Storage &) = delete;
    Storage(Storage &&) = delete;
    Storage &operator=(Storage &&) = delete;

    /**
     * @brief Returns the number of points currently in the storage.
     *
     * @return std::size_t
     */
    std::size_t point_count() const;

    pdal::SpatialReference spatial_reference() const;

    std::pair<std::vector<pdal::Dimension::Id>,
              std::vector<ProprietaryDimension>>
    dimensions() const;

    OGREnvelopePtr bounding_box() const;
    Bbox_2 bounding_box_cgal() const;

    // Getters for internal PDAL objects
    pdal::PointViewPtr get_view() const { return view; }
    pdal::PointTable &get_table() { return *table; }
    const pdal::PointTable &get_table() const { return *table; }

    /**
     * @brief Get the value of an attribute for a point in the storage.
     *
     * @tparam T The type of the value to get. Must match the type of the
     * dimension.
     * @param dim The dimension to get the value for. Must be registered in the
     * storage's PointTable.
     * @param idx The index of the point to get the attribute for. Must be
     * between 0 (inclusive) and the current point count (exclusive).
     * @return T The value of the attribute for the given point.
     */
    template <typename T>
    T get_field_as(pdal::Dimension::Id dim, PointId point_id) const {
        return view->getFieldAs<T>(dim, point_id);
    }

    /**
     * @brief Get the value of an attribute for a point in the storage.
     *
     * @tparam T The type of the value to get. Must match the type of the
     * dimension.
     * @param dim The dimension to get the value for. Must be registered in the
     * storage's PointTable.
     * @param idx The index of the point to get the attribute for. Must be
     * between 0 (inclusive) and the current point count (exclusive).
     * @return T The value of the attribute for the given point.
     */
    template <typename T>
    T get_field_as(ProprietaryDimension dim, PointId point_id) const {
        return view->getFieldAs<T>(
            table->layout()->findProprietaryDim(dim.name), point_id);
    }

    /**
     * @brief Set the value of an attribute for a point in the storage.
     *
     * @warning This fails if the given index is strictly higher than the
     * current point count.
     *
     * @tparam T The type of the value to set. Must match the type of the
     * dimension.
     * @param dim The dimension to set. Must be registered in the storage's
     * PointTable.
     * @param idx The index of the point to set the attribute for. Must be
     * between 0 and the current point count (inclusive).
     * @param value The value to set for the attribute. Must be of the same type
     * as the dimension.
     */
    template <typename T>
    void set_field(pdal::Dimension::Id dim, PointId point_id, const T value) {
        view->setField(dim, point_id, value);
    }

    /**
     * @brief Set the value of an attribute for a point in the storage.
     *
     * @warning This fails if the given index is strictly higher than the
     * current point count.
     *
     * @tparam T The type of the value to set. Must match the type of the
     * dimension.
     * @param dim The dimension to set. Must be registered in the storage's
     * PointTable.
     * @param idx The index of the point to set the attribute for. Must be
     * between 0 and the current point count (inclusive).
     * @param value The value to set for the attribute. Must be of the same type
     * as the dimension.
     */
    template <typename T>
    void set_field(ProprietaryDimension dim, PointId point_id, const T value) {
        view->setField(table->layout()->findProprietaryDim(dim.name), point_id,
                       value);
    }

    template <typename T>
    void copy_field(pdal::Dimension::Id dim, PointId new_point_id,
                    std::shared_ptr<Storage> other, PointId other_point_id) {
        T value = other->get_field_as<T>(dim, other_point_id);
        this->set_field(dim, new_point_id, value);
    }

    template <typename T>
    void copy_field(ProprietaryDimension dim, PointId new_point_id,
                    std::shared_ptr<Storage> other, PointId other_point_id) {
        T value = other->get_field_as<T>(dim, other_point_id);
        this->set_field(dim, new_point_id, value);
    }

    void set_point(PointId point_id, const Point_3 &point) {
        this->set_field(pdal::Dimension::Id::X, point_id, point.x());
        this->set_field(pdal::Dimension::Id::Y, point_id, point.y());
        this->set_field(pdal::Dimension::Id::Z, point_id, point.z());
    }

    Point_2 get_point_2d(PointId point_id) const {
        // double x = view->getFieldAs<double>(pdal::Dimension::Id::X,
        // point_id); double y =
        // view->getFieldAs<double>(pdal::Dimension::Id::Y, point_id); return
        // Point_2(x, y);
        Point_3 point_3d = get_point(point_id);
        return Point_2(point_3d.x(), point_3d.y());
    }

    Point_3 get_point(PointId point_id) const {
        // double x = view->getFieldAs<double>(pdal::Dimension::Id::X,
        // point_id); double y =
        // view->getFieldAs<double>(pdal::Dimension::Id::Y, point_id); double z
        // = view->getFieldAs<double>(pdal::Dimension::Id::Z, point_id); return
        // Point_3(x, y, z);
        return cached_points[point_id];
    }

    void cache_points();

    void build_kd_tree_2d();
    std::shared_ptr<KdTree_2> get_kd_tree_2d() const;
    void build_kd_tree_3d();
    std::shared_ptr<KdTree_3> get_kd_tree_3d() const;
};

typedef std::shared_ptr<Storage> StoragePtr;

struct Ray3D {
  private:
    Point_3 origin;
    double gps_time;
    uint8_t scan_direction_flag;
    double scan_angle;
    std::vector<PointId>
        return_number_to_point_id; // The point IDs in order of return
                                   // number from lowest to highest
    std::map<PointId, uint8_t> point_id_to_return_number; // Mapping from point
                                                          // ID to return
                                                          // number

  public:
    Ray3D(const Point_3 &origin_, double gps_time_,
          uint8_t scan_direction_flag_, double scan_angle_,
          const std::vector<PointId> &point_ids_,
          const std::vector<int> &return_numbers);

    bool empty() const { return return_number_to_point_id.empty(); }
    std::size_t size() const { return return_number_to_point_id.size(); }

    const Point_3 &get_origin() const { return origin; }
    double get_gps_time() const { return gps_time; }
    uint8_t get_scan_direction_flag() const { return scan_direction_flag; }
    double get_scan_angle() const { return scan_angle; }
    const std::vector<PointId> &get_point_ids() const {
        return return_number_to_point_id;
    }
    uint8_t get_return_number(PointId point_id) const {
        auto it = point_id_to_return_number.find(point_id);
        if (it == point_id_to_return_number.end()) {
            throw std::out_of_range(
                "Point ID not found in return number mapping: " +
                std::to_string(point_id));
        }
        return it->second;
    }
    uint8_t get_number_of_returns() const {
        return return_number_to_point_id.size();
    }
    /**
     * @brief Get the ID of the point with the given index in order of return
     * number.
     * The index is between 0 and the number of returns (exclusive). The point
     * with index 0 is the point with the lowest return number, and the point
     * with index number_of_returns - 1 is the point with the highest return
     * number. The given index can also be negative to count from the highest
     * return number.
     *
     * @param index_return_number The index of the point in order of return
     * number.
     * @return PointId The ID of the point at the given index.
     */
    PointId get_point_id_in_return_order(int index_return_number) const;

    CustomCGAL::Angle angle_to(const Ray3D &other, StoragePtr points) const;
};

const double SCAN_LINE_MAX_GPS_TIME_DIFFERENCE = 1e-4;

struct ScanLine3D {
  private:
    std::vector<RayId> ray_ids;
    std::map<RayId, std::size_t>
        ray_id_to_index; // Mapping from ray ID to its index in the scan line
    std::map<double, RayId> scan_angle_to_ray_id; // Mapping from scan angle to
                                                  // ray ID for quick access

    std::shared_ptr<std::vector<Ray3D>> rays;
    StoragePtr points;

  public:
    ScanLine3D(StoragePtr points_, std::shared_ptr<std::vector<Ray3D>> rays_,
               const std::vector<RayId> &ray_ids_);

    const std::vector<RayId> &get_ray_ids() const { return ray_ids; }

    std::optional<RayId> get_next_ray_id(std::optional<RayId> ray_id) const;
    std::optional<RayId> get_prev_ray_id(std::optional<RayId> ray_id) const;
    RayId get_closest_ray_by_scan_angle(double scan_angle) const;
    RayId get_closest_ray_by_direction(RayId other_ray_id) const;
    RayId get_closest_ray_by_two_directions(const RayId other_ray_id_1,
                                            const RayId other_ray_id_2) const;
};

struct Topology3D {
  private:
    std::vector<Ray3D> rays;
    std::vector<RayId> point_id_to_ray_id;

    std::vector<ScanLine3D> scan_lines;
    std::vector<ScanLineId> ray_id_to_scan_line_id;

    std::map<double, RayId> gps_time_to_ray_id;
    std::vector<RayId> rays_gps_time_order;
    std::map<RayId, std::size_t> ray_id_to_gps_time_order_index;
    // std::vector<RayId> map_next_ray_gps_time_order;
    // std::vector<RayId> map_prev_ray_gps_time_order;
    std::vector<RayId> map_next_ray_vehicle_axis_order;
    std::vector<RayId> map_prev_ray_vehicle_axis_order;

    void init(Trajectory trajectory);

  public:
    StoragePtr points;

    Topology3D(std::vector<pdal::Dimension::Id> predefined_dims,
               std::vector<ProprietaryDimension> proprietary_dims,
               pdal::SpatialReference spatial_ref, Trajectory trajectory);
    Topology3D(pdal::PointViewPtr view, pdal::SpatialReference spatial_ref,
               Trajectory trajectory);
    Topology3D(StoragePtr storage, Trajectory trajectory);

    std::size_t ray_count() const { return rays.size(); }

    // const std::vector<Ray3D> &get_rays() const;
    // const std::vector<RayId> &get_rays_in_gps_time_order() const;
    RayId get_ray_id(PointId point_id) const;
    const Ray3D &get_ray(RayId i) const;

    ScanLineId get_scan_line_id(RayId ray_id) const;
    ScanLineId get_scan_line_id(PointId point_id) const;
    const ScanLine3D &get_scan_line(ScanLineId i) const;

    RayId get_first_ray_in_gps_time_order() const {
        return rays_gps_time_order[0];
    }
    std::optional<RayId> get_next_ray_in_gps_time_order(RayId i) const;
    std::optional<RayId> get_prev_ray_in_gps_time_order(RayId i) const;

    std::optional<RayId> get_next_ray_in_vehicle_line(RayId i) const;
    std::optional<RayId> get_prev_ray_in_vehicle_line(RayId i) const;

    std::optional<ScanLineId>
    get_next_scan_line_id(std::optional<ScanLineId> scan_line_id) const;
    std::optional<ScanLineId>
    get_prev_scan_line_id(std::optional<ScanLineId> scan_line_id) const;

    CustomCGAL::Angle angle_between(RayId ray_1, RayId ray_2) const;
    Point_3 get_point_at_height(RayId ray_id, double height) const;
};
} // namespace PtsStructs