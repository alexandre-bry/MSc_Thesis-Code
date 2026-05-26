#pragma once

#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "../utils/cgal.hpp"

namespace PointSelection {

struct RoofSegment {
    Segment_2 segment;
    Bbox_2 bbox;
};

struct RoofFace {
    std::vector<Point_3> outer_ring_3d;
    std::vector<Point_2> outer_ring_2d;
    std::vector<std::vector<Point_3>> holes_3d;
    std::vector<std::vector<Point_2>> holes_2d;
    std::vector<RoofSegment> boundary_segments;
    std::optional<Bbox_2> bbox;
    std::optional<Plane_3> plane;
    double min_z = 0.0;
    double max_z = 0.0;

    /** @brief Return true if this face has valid cached geometry. */
    bool is_valid() const;

    /** @brief Test whether an XY point lies within the face (outer ring minus
     * holes).
     *  @param point XY point to test
     *  @return true when the point is inside the face footprint or on its
     * boundary
     */
    bool contains_xy(const Point_2 &point) const;

    /** @brief Compute the roof height at the given XY point using the cached
     * face plane.
     *  @param point XY point to evaluate
     *  @return the Z coordinate on the roof plane at the XY position, or
     * std::nullopt if the face plane is degenerate
     */
    std::optional<double> roof_height_at(const Point_2 &point) const;

    /** @brief Compute the roof height at the closest boundary point to `point`.
     *  This implements the horizontal "halo" behaviour: if `point` lies outside
     * the face footprint but within `horizontal_buffer` of a boundary segment,
     * the height at the closest projected boundary point is returned.
     *  @param point XY point to evaluate
     *  @param horizontal_buffer maximum horizontal search distance
     *  @return the roof height at the closest boundary projection, or
     * std::nullopt
     */
    std::optional<double>
    roof_height_at_closest_boundary_point(const Point_2 &point,
                                          double horizontal_buffer) const;
};

struct RoofBuilding {
    std::string id;
    std::vector<RoofFace> roof_faces;
    std::optional<Bbox_2> bbox;
    double min_z = 0.0;
    double max_z = 0.0;

    /** @brief Return true when this building has at least one roof face. */
    bool is_valid() const;

    /** @brief Test whether a 3D point is under this building's roof.
     *  The test follows the coarse-to-fine strategy: building bbox -> face bbox
     *  -> face polygon -> plane height; it also supports a horizontal halo
     *  around face boundaries.
     *  @param point 3D point to test
     *  @param vertical_buffer allowed vertical tolerance
     *  @param horizontal_buffer allowed horizontal halo
     *  @return true when the point is considered under the roof
     */
    bool contains(const Point_3 &point, double vertical_buffer,
                  double horizontal_buffer) const;
};

class RoofSelectionStore {
  public:
    /** @brief Find a building by id (const). Returns std::nullopt when missing.
     */
    std::optional<std::reference_wrapper<const RoofBuilding>>
    find_building(const std::string &building_id) const;

    /** @brief Find a building by id (mutable). Returns std::nullopt when
     * missing. */
    std::optional<std::reference_wrapper<RoofBuilding>>
    find_building(const std::string &building_id);
    const std::unordered_map<std::string, RoofBuilding> &buildings() const {
        return buildings_;
    }

    /** @brief Test whether a 3D point is under the roof of a building
     * identified by its id.
     *  @param building_id id of the building to check
     *  @param point 3D point to test
     *  @param vertical_buffer allowed vertical tolerance
     *  @param horizontal_buffer allowed horizontal halo
     *  @return true when the point is considered under the roof
     */
    bool contains(const std::string &building_id, const Point_3 &point,
                  double vertical_buffer, double horizontal_buffer) const;

    /** @brief Test whether a 3D point is under the roof of the provided
     * building. This is a thin forwarder to `RoofBuilding::contains`.
     */
    bool contains(const RoofBuilding &building, const Point_3 &point,
                  double vertical_buffer, double horizontal_buffer) const;

  private:
    std::unordered_map<std::string, RoofBuilding> buildings_;

    friend RoofSelectionStore
    read_cityjson_roofs(const std::string &cityjson_path);
};

struct SelectPointsUnderRoofsOptions {
    std::string input_points_file;
    std::string input_roofs_file;
    std::string output_points_file;
    double vertical_buffer = 0.2;
    double horizontal_buffer = 0.2;
    bool overwrite = false;
};

/** @brief Read a CityJSON file and collect all `RoofSurface` faces grouped by
 *  their root building id.
 *  @param cityjson_path path to the CityJSON file
 *  @return a populated `RoofSelectionStore`
 */
RoofSelectionStore read_cityjson_roofs(const std::string &cityjson_path);

/** @brief Convenience free-function to test a point against a building id in
 * the provided store.
 */
bool point_is_under_roof(const RoofSelectionStore &store,
                         const std::string &building_id, const Point_3 &point,
                         double vertical_buffer, double horizontal_buffer);

/** @brief Convenience overload to test a point against a `RoofBuilding`
 * directly.
 */
bool point_is_under_roof(const RoofBuilding &building, const Point_3 &point,
                         double vertical_buffer, double horizontal_buffer);

/** @brief Read a point cloud and keep only points under at least one roof.
 *  The roof geometry is loaded from a CityJSON file and points are written to a
 *  new LAS/LAZ output while preserving all input dimensions.
 *  @param input_points_file input LAS/LAZ file
 *  @param input_roofs_file input CityJSON file containing roofs
 *  @param output_points_file output LAS/LAZ file with selected points
 *  @param vertical_buffer allowed vertical tolerance under roof surfaces
 *  @param horizontal_buffer allowed horizontal halo around roof boundaries
 *  @param overwrite overwrite output file when it already exists
 */
void select_points_under_roofs(const std::string &input_points_file,
                               const std::string &input_roofs_file,
                               const std::string &output_points_file,
                               double vertical_buffer, double horizontal_buffer,
                               bool overwrite);

} // namespace PointSelection
