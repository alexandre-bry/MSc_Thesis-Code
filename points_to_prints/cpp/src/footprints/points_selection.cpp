#include "points_selection.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <pdal/Dimension.hpp>

#include "json/json.hpp"

#include "../las/reader.hpp"
#include "../las/writer.hpp"
#include "../utils/pbar.hpp"

using json = nlohmann::json;

namespace PointSelection {
namespace {

constexpr double kGeometryEpsilon = 1e-9;
constexpr double faceMatchTolerance = 0.2;

struct CityObjectMeta {
    std::string type;
    std::vector<std::string> parents;
};

bool is_polygon_node(const json &node) {
    return node.is_array() && !node.empty() && node[0].is_array() &&
           !node[0].empty() && node[0][0].is_number_integer();
}

bool is_ring_node(const json &node) {
    return node.is_array() && !node.empty() && node[0].is_number_integer();
}

bool points_are_close(const Point_2 &a, const Point_2 &b) {
    return std::abs(a.x() - b.x()) <= kGeometryEpsilon &&
           std::abs(a.y() - b.y()) <= kGeometryEpsilon;
}

bool point_on_segment(const Point_2 &point, const Point_2 &a,
                      const Point_2 &b) {
    const double ab_x = b.x() - a.x();
    const double ab_y = b.y() - a.y();
    const double ap_x = point.x() - a.x();
    const double ap_y = point.y() - a.y();
    const double cross = ab_x * ap_y - ab_y * ap_x;
    if (std::abs(cross) > kGeometryEpsilon) {
        return false;
    }

    const double dot = ap_x * ab_x + ap_y * ab_y;
    if (dot < -kGeometryEpsilon) {
        return false;
    }

    const double ab_len_sq = ab_x * ab_x + ab_y * ab_y;
    if (dot - ab_len_sq > kGeometryEpsilon) {
        return false;
    }

    return true;
}

bool point_on_ring_boundary(const std::vector<Point_2> &ring,
                            const Point_2 &point) {
    if (ring.size() < 2) {
        return false;
    }

    for (std::size_t i = 0; i < ring.size(); ++i) {
        const Point_2 &a = ring[i];
        const Point_2 &b = ring[(i + 1) % ring.size()];
        if (point_on_segment(point, a, b)) {
            return true;
        }
    }

    return false;
}

bool point_in_ring(const std::vector<Point_2> &ring, const Point_2 &point) {
    if (ring.size() < 3) {
        return false;
    }

    if (point_on_ring_boundary(ring, point)) {
        return true;
    }

    bool inside = false;
    for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
        const Point_2 &pi = ring[i];
        const Point_2 &pj = ring[j];
        const bool intersects =
            ((pi.y() > point.y()) != (pj.y() > point.y())) &&
            (point.x() <
             (pj.x() - pi.x()) * (point.y() - pi.y()) / (pj.y() - pi.y()) +
                 pi.x());
        if (intersects) {
            inside = !inside;
        }
    }

    return inside;
}

struct ProjectionResult {
    double distance_sq;
    Point_2 projected;
};

ProjectionResult project_point_to_segment(const Point_2 &point,
                                          const Point_2 &a, const Point_2 &b) {
    const double ab_x = b.x() - a.x();
    const double ab_y = b.y() - a.y();
    const double ap_x = point.x() - a.x();
    const double ap_y = point.y() - a.y();
    const double ab_len_sq = ab_x * ab_x + ab_y * ab_y;

    double t = 0.0;
    if (ab_len_sq > 0.0) {
        t = (ap_x * ab_x + ap_y * ab_y) / ab_len_sq;
        t = std::clamp(t, 0.0, 1.0);
    }

    const Point_2 projected(a.x() + t * ab_x, a.y() + t * ab_y);
    const double dx = point.x() - projected.x();
    const double dy = point.y() - projected.y();
    return ProjectionResult{dx * dx + dy * dy, projected};
}

Bbox_2 make_bbox_from_points(const std::vector<Point_2> &points) {
    if (points.empty()) {
        throw std::runtime_error("Cannot build a bbox from an empty ring");
    }

    double xmin = points.front().x();
    double xmax = points.front().x();
    double ymin = points.front().y();
    double ymax = points.front().y();
    for (const auto &point : points) {
        xmin = std::min(xmin, point.x());
        xmax = std::max(xmax, point.x());
        ymin = std::min(ymin, point.y());
        ymax = std::max(ymax, point.y());
    }
    return Bbox_2(xmin, ymin, xmax, ymax);
}

void grow_bbox(std::optional<Bbox_2> &bbox, const Bbox_2 &candidate) {
    if (!bbox.has_value()) {
        bbox = candidate;
        return;
    }

    bbox = Bbox_2(std::min(bbox->xmin(), candidate.xmin()),
                  std::min(bbox->ymin(), candidate.ymin()),
                  std::max(bbox->xmax(), candidate.xmax()),
                  std::max(bbox->ymax(), candidate.ymax()));
}

void grow_bbox(std::optional<Bbox_2> &bbox, const Point_2 &point) {
    grow_bbox(bbox, Bbox_2(point.x(), point.y(), point.x(), point.y()));
}

std::vector<Point_3> decode_ring(const json &ring_node,
                                 const std::vector<Point_3> &vertices) {
    if (!is_ring_node(ring_node)) {
        throw std::runtime_error("Expected a ring node while decoding a roof");
    }

    std::vector<Point_3> ring;
    ring.reserve(ring_node.size());
    for (const auto &vertex_index_value : ring_node) {
        const std::size_t vertex_index = vertex_index_value.get<std::size_t>();
        if (vertex_index >= vertices.size()) {
            throw std::out_of_range("CityJSON vertex index out of range");
        }
        ring.push_back(vertices[vertex_index]);
    }

    if (ring.size() >= 2 &&
        points_are_close(Point_2(ring.front().x(), ring.front().y()),
                         Point_2(ring.back().x(), ring.back().y()))) {
        ring.pop_back();
    }

    return ring;
}

std::optional<Plane_3> make_plane_from_ring(const std::vector<Point_3> &ring) {
    if (ring.size() < 3) {
        return std::nullopt;
    }

    for (std::size_t i = 0; i + 2 < ring.size(); ++i) {
        for (std::size_t j = i + 1; j + 1 < ring.size(); ++j) {
            for (std::size_t k = j + 1; k < ring.size(); ++k) {
                if (!CGAL::collinear(ring[i], ring[j], ring[k])) {
                    return Plane_3(ring[i], ring[j], ring[k]);
                }
            }
        }
    }

    return std::nullopt;
}

RoofFace build_roof_face(const json &polygon_node,
                         const std::vector<Point_3> &vertices) {
    RoofFace face;

    if (!is_polygon_node(polygon_node)) {
        throw std::runtime_error(
            "Expected a polygon node while decoding a roof");
    }

    for (std::size_t ring_idx = 0; ring_idx < polygon_node.size(); ++ring_idx) {
        std::vector<Point_3> ring_3d =
            decode_ring(polygon_node[ring_idx], vertices);
        if (ring_3d.size() < 3) {
            throw std::runtime_error(
                "Roof ring has fewer than three unique points");
        }

        std::vector<Point_2> ring_2d;
        ring_2d.reserve(ring_3d.size());
        for (const auto &point : ring_3d) {
            ring_2d.emplace_back(point.x(), point.y());
            grow_bbox(face.bbox, ring_2d.back());
        }

        double local_min_z = ring_3d.front().z();
        double local_max_z = ring_3d.front().z();
        for (const auto &point : ring_3d) {
            local_min_z = std::min(local_min_z, point.z());
            local_max_z = std::max(local_max_z, point.z());
        }

        if (ring_idx == 0) {
            face.outer_ring_3d = std::move(ring_3d);
            face.outer_ring_2d = std::move(ring_2d);
            face.min_z = local_min_z;
            face.max_z = local_max_z;
        } else {
            face.holes_3d.push_back(std::move(ring_3d));
            face.holes_2d.push_back(std::move(ring_2d));
            face.min_z = std::min(face.min_z, local_min_z);
            face.max_z = std::max(face.max_z, local_max_z);
        }
    }

    face.plane = make_plane_from_ring(face.outer_ring_3d);
    if (!face.plane.has_value()) {
        throw std::runtime_error(
            "Roof face is degenerate and has no valid plane");
    }

    auto append_segments = [&face](const std::vector<Point_2> &ring) {
        if (ring.size() < 2) {
            return;
        }
        for (std::size_t i = 0; i < ring.size(); ++i) {
            const Point_2 &a = ring[i];
            const Point_2 &b = ring[(i + 1) % ring.size()];
            Segment_2 segment(a, b);
            face.boundary_segments.push_back(
                RoofSegment{segment, segment.bbox()});
        }
    };

    append_segments(face.outer_ring_2d);
    for (const auto &hole_ring : face.holes_2d) {
        append_segments(hole_ring);
    }

    if (!face.bbox.has_value()) {
        throw std::runtime_error("Failed to build roof face bbox");
    }

    return face;
}

void collect_roof_faces(const json &boundary_node, const json &semantic_node,
                        const std::vector<Point_3> &vertices,
                        const std::vector<bool> &roof_surface_flags,
                        RoofBuilding &building) {
    if (!boundary_node.is_array() || boundary_node.empty()) {
        return;
    }

    if (is_polygon_node(boundary_node)) {
        int surface_index = -1;
        if (semantic_node.is_number_integer()) {
            surface_index = semantic_node.get<int>();
        }

        if (surface_index < 0 ||
            static_cast<std::size_t>(surface_index) >=
                roof_surface_flags.size() ||
            !roof_surface_flags[static_cast<std::size_t>(surface_index)]) {
            return;
        }

        RoofFace face = build_roof_face(boundary_node, vertices);
        grow_bbox(building.bbox, *face.bbox);
        building.min_z = std::min(building.min_z, face.min_z);
        building.max_z = std::max(building.max_z, face.max_z);
        building.roof_faces.push_back(std::move(face));
        return;
    }

    for (std::size_t i = 0; i < boundary_node.size(); ++i) {
        const json &child_boundary = boundary_node[i];
        const json child_semantic =
            (semantic_node.is_array() && i < semantic_node.size())
                ? semantic_node[i]
                : json();
        collect_roof_faces(child_boundary, child_semantic, vertices,
                           roof_surface_flags, building);
    }
}

std::string resolve_root_building_id(
    const std::string &object_id,
    const std::unordered_map<std::string, CityObjectMeta> &city_objects) {
    std::string current_id = object_id;
    std::unordered_map<std::string, bool> visited;

    while (true) {
        auto it = city_objects.find(current_id);
        if (it == city_objects.end() || it->second.parents.empty()) {
            return current_id;
        }

        const std::string &parent_id = it->second.parents.front();
        if (visited.find(parent_id) != visited.end()) {
            return current_id;
        }

        visited[current_id] = true;
        current_id = parent_id;
    }
}

std::vector<Point_3> read_vertices(const json &root) {
    std::vector<Point_3> vertices;
    if (!root.contains("vertices") || !root["vertices"].is_array()) {
        throw std::runtime_error("CityJSON file does not contain vertices");
    }

    std::array<double, 3> scale{1.0, 1.0, 1.0};
    std::array<double, 3> translate{0.0, 0.0, 0.0};
    if (root.contains("transform") && root["transform"].is_object()) {
        const json &transform = root["transform"];
        if (transform.contains("scale") && transform["scale"].is_array() &&
            transform["scale"].size() == 3) {
            for (std::size_t i = 0; i < 3; ++i) {
                scale[i] = transform["scale"][i].get<double>();
            }
        }
        if (transform.contains("translate") &&
            transform["translate"].is_array() &&
            transform["translate"].size() == 3) {
            for (std::size_t i = 0; i < 3; ++i) {
                translate[i] = transform["translate"][i].get<double>();
            }
        }
    }

    vertices.reserve(root["vertices"].size());
    for (const auto &vertex : root["vertices"]) {
        if (!vertex.is_array() || vertex.size() < 3) {
            throw std::runtime_error("Invalid CityJSON vertex entry");
        }
        const double x = vertex[0].get<double>() * scale[0] + translate[0];
        const double y = vertex[1].get<double>() * scale[1] + translate[1];
        const double z = vertex[2].get<double>() * scale[2] + translate[2];
        vertices.emplace_back(x, y, z);
    }

    return vertices;
}

bool point_matches_roof_face(const RoofFace &face, const Point_3 &point,
                             double vertical_buffer, double horizontal_buffer) {
    const Point_2 point_2d(point.x(), point.y());

    // If the point is within the 2D projection of the face
    if (face.contains_xy(point_2d)) {
        const std::optional<double> roof_height = face.roof_height_at(point_2d);
        return roof_height.has_value() &&
               point.z() <= *roof_height - vertical_buffer;
    }

    // If the point is outside the 2D projection
    const std::optional<double> roof_height =
        face.roof_height_at_closest_boundary_point(point_2d, horizontal_buffer);
    return roof_height.has_value() &&
           point.z() <= *roof_height - vertical_buffer;
}

} // namespace

bool RoofFace::is_valid() const {
    return plane.has_value() && bbox.has_value() && !outer_ring_2d.empty();
}

bool RoofFace::contains_xy(const Point_2 &point) const {
    if (!is_valid()) {
        return false;
    }

    if (!point_in_ring(outer_ring_2d, point)) {
        return false;
    }

    for (const auto &hole : holes_2d) {
        if (point_in_ring(hole, point)) {
            return false;
        }
    }

    return true;
}

double RoofFace::distance_xy(const Point_2 &point) const {
    if (!is_valid()) {
        return std::numeric_limits<double>::infinity();
    }

    if (contains_xy(point)) {
        return 0.0;
    }

    double best_distance_sq = std::numeric_limits<double>::infinity();
    for (const auto &segment_data : boundary_segments) {
        const Bbox_2 &segment_bbox = segment_data.bbox;
        if (point.x() < segment_bbox.xmin() - kGeometryEpsilon ||
            point.x() > segment_bbox.xmax() + kGeometryEpsilon ||
            point.y() < segment_bbox.ymin() - kGeometryEpsilon ||
            point.y() > segment_bbox.ymax() + kGeometryEpsilon) {
            continue;
        }

        const ProjectionResult pr =
            project_point_to_segment(point, segment_data.segment.source(),
                                     segment_data.segment.target());
        if (pr.distance_sq < best_distance_sq) {
            best_distance_sq = pr.distance_sq;
        }
    }

    return std::sqrt(best_distance_sq);
}

std::optional<double> RoofFace::roof_height_at(const Point_2 &point) const {
    if (!plane.has_value()) {
        return std::nullopt;
    }

    const double a = plane->a();
    const double b = plane->b();
    const double c = plane->c();
    const double d = plane->d();

    if (std::abs(c) <= kGeometryEpsilon) {
        return std::nullopt;
    }

    return -(a * point.x() + b * point.y() + d) / c;
}

std::optional<double> RoofFace::roof_height_at_closest_boundary_point(
    const Point_2 &point, double horizontal_buffer) const {
    if (!is_valid() || boundary_segments.empty()) {
        return std::nullopt;
    }

    const double max_distance_sq = horizontal_buffer * horizontal_buffer;
    double best_distance_sq = std::numeric_limits<double>::infinity();
    Point_2 closest_point;

    for (const auto &segment_data : boundary_segments) {
        const Bbox_2 &segment_bbox = segment_data.bbox;
        if (point.x() < segment_bbox.xmin() - horizontal_buffer ||
            point.x() > segment_bbox.xmax() + horizontal_buffer ||
            point.y() < segment_bbox.ymin() - horizontal_buffer ||
            point.y() > segment_bbox.ymax() + horizontal_buffer) {
            continue;
        }

        const ProjectionResult pr =
            project_point_to_segment(point, segment_data.segment.source(),
                                     segment_data.segment.target());
        if (pr.distance_sq < best_distance_sq) {
            best_distance_sq = pr.distance_sq;
            closest_point = pr.projected;
        }
    }

    if (best_distance_sq > max_distance_sq) {
        return std::nullopt;
    }

    return roof_height_at(closest_point);
}

bool RoofBuilding::is_valid() const { return !roof_faces.empty(); }

std::optional<std::reference_wrapper<const RoofBuilding>>
RoofSelectionStore::find_building(const std::string &building_id) const {
    auto it = buildings_.find(building_id);
    if (it == buildings_.end()) {
        return std::nullopt;
    }
    return std::cref(it->second);
}

std::optional<std::reference_wrapper<RoofBuilding>>
RoofSelectionStore::find_building(const std::string &building_id) {
    auto it = buildings_.find(building_id);
    if (it == buildings_.end()) {
        return std::nullopt;
    }
    return std::ref(it->second);
}

bool RoofSelectionStore::contains(const std::string &building_id,
                                  const Point_3 &point, double vertical_buffer,
                                  double horizontal_buffer) const {
    const auto maybe = find_building(building_id);
    if (!maybe.has_value()) {
        return false;
    }
    return contains(maybe->get(), point, vertical_buffer, horizontal_buffer);
}

bool RoofSelectionStore::contains(const RoofBuilding &building,
                                  const Point_3 &point, double vertical_buffer,
                                  double horizontal_buffer) const {
    return building.contains(point, vertical_buffer, horizontal_buffer);
}

bool RoofBuilding::contains(const Point_3 &point, double vertical_buffer,
                            double horizontal_buffer) const {
    if (!is_valid()) {
        return false;
    }

    if (bbox.has_value()) {
        if (point.x() < bbox->xmin() - horizontal_buffer ||
            point.x() > bbox->xmax() + horizontal_buffer ||
            point.y() < bbox->ymin() - horizontal_buffer ||
            point.y() > bbox->ymax() + horizontal_buffer) {
            return false;
        }
    }

    if (point.z() > max_z - vertical_buffer) {
        return false;
    }

    // Find the faces that the point belongs to or that it is close to, and
    // check the point against them
    const Point_2 point_2d(point.x(), point.y());
    std::vector<RoofFace> faces_to_check;

    for (const auto &face : roof_faces) {
        if (!face.is_valid()) {
            continue;
        }

        if (face.bbox.has_value()) {
            if (point.x() < face.bbox->xmin() - horizontal_buffer ||
                point.x() > face.bbox->xmax() + horizontal_buffer ||
                point.y() < face.bbox->ymin() - horizontal_buffer ||
                point.y() > face.bbox->ymax() + horizontal_buffer) {
                continue;
            }
        }

        if (face.contains_xy(point_2d)) {
            faces_to_check.push_back(face);
            continue;
        }

        const double distance = face.distance_xy(point_2d);
        if (distance < horizontal_buffer) {
            faces_to_check.push_back(face);
        }
    }

    if (faces_to_check.empty()) {
        return false;
    }

    for (const auto &face : faces_to_check) {
        if (point_matches_roof_face(face, point, vertical_buffer,
                                    horizontal_buffer)) {
            return true;
        }
    }

    return false;
}

void RoofBuilding::find_faces_for_segment(
    const Segment_2 &segment, const double overlap_threshold,
    std::vector<std::reference_wrapper<const RoofFace>> &matching_faces) const {
    if (!is_valid()) {
        return;
    }

    const double overlap_threshold_sq = overlap_threshold * overlap_threshold;

    for (const auto &face : roof_faces) {
        if (!face.is_valid()) {
            continue;
        }

        // Find the closest point on the segment to the face's 2D projection
        const Point_2 segment_start = segment.source();
        const Point_2 segment_end = segment.target();
        const Line_2 segment_line(segment_start, segment_end);

        double smallest_distance_sq = std::numeric_limits<double>::infinity();
        for (const auto &face_segment_data : face.boundary_segments) {
            const Point_2 face_segment_start =
                face_segment_data.segment.source();
            const Point_2 face_segment_end = face_segment_data.segment.target();

            const Point_2 face_segment_start_proj =
                segment_line.projection(face_segment_start);
            const Point_2 face_segment_end_proj =
                segment_line.projection(face_segment_end);

            const double distance_start_sq = CGAL::squared_distance(
                face_segment_start_proj, face_segment_start);
            const double distance_end_sq =
                CGAL::squared_distance(face_segment_end_proj, face_segment_end);
            const double max_distance_sq =
                std::max(distance_start_sq, distance_end_sq);

            if (max_distance_sq < smallest_distance_sq) {
                smallest_distance_sq = max_distance_sq;
            }
        }

        if (smallest_distance_sq < overlap_threshold_sq) {
            matching_faces.push_back(std::cref(face));
        }
    }
}

RoofSelectionStore read_cityjson_roofs(const std::string &cityjson_path) {
    std::ifstream input(cityjson_path);
    if (!input.is_open()) {
        throw std::runtime_error("Failed to open CityJSON file: " +
                                 cityjson_path);
    }

    json root;
    input >> root;

    const std::vector<Point_3> vertices = read_vertices(root);

    if (!root.contains("CityObjects") || !root["CityObjects"].is_object()) {
        throw std::runtime_error("CityJSON file does not contain CityObjects");
    }

    std::unordered_map<std::string, CityObjectMeta> city_objects;
    for (auto it = root["CityObjects"].begin(); it != root["CityObjects"].end();
         ++it) {
        CityObjectMeta meta;
        const json &city_object = it.value();

        if (city_object.contains("type") && city_object["type"].is_string()) {
            meta.type = city_object["type"].get<std::string>();
        }

        if (city_object.contains("parents") &&
            city_object["parents"].is_array()) {
            for (const auto &parent : city_object["parents"]) {
                if (parent.is_string()) {
                    meta.parents.push_back(parent.get<std::string>());
                }
            }
        }

        city_objects.emplace(it.key(), std::move(meta));
    }

    RoofSelectionStore store;

    for (auto it = root["CityObjects"].begin(); it != root["CityObjects"].end();
         ++it) {
        const std::string &object_id = it.key();
        const json &city_object = it.value();

        if (!city_object.contains("geometry") ||
            !city_object["geometry"].is_array()) {
            continue;
        }

        const std::string root_building_id =
            resolve_root_building_id(object_id, city_objects);
        RoofBuilding &building = store.buildings_[root_building_id];
        building.id = root_building_id;

        for (const auto &geometry : city_object["geometry"]) {
            if (!geometry.is_object() || !geometry.contains("boundaries") ||
                !geometry["boundaries"].is_array()) {
                continue;
            }

            std::vector<bool> roof_surface_flags;
            if (geometry.contains("semantics") &&
                geometry["semantics"].is_object() &&
                geometry["semantics"].contains("surfaces") &&
                geometry["semantics"]["surfaces"].is_array()) {
                const auto &surfaces = geometry["semantics"]["surfaces"];
                roof_surface_flags.resize(surfaces.size(), false);
                for (std::size_t i = 0; i < surfaces.size(); ++i) {
                    if (surfaces[i].is_object() &&
                        surfaces[i].contains("type") &&
                        surfaces[i]["type"].is_string() &&
                        surfaces[i]["type"].get<std::string>() ==
                            "RoofSurface") {
                        roof_surface_flags[i] = true;
                    }
                }
            }

            if (roof_surface_flags.empty()) {
                continue;
            }

            const json &semantics_values =
                (geometry.contains("semantics") &&
                 geometry["semantics"].is_object() &&
                 geometry["semantics"].contains("values"))
                    ? geometry["semantics"]["values"]
                    : json();

            collect_roof_faces(geometry["boundaries"], semantics_values,
                               vertices, roof_surface_flags, building);
        }
    }

    for (auto &[building_id, building] : store.buildings_) {
        building.id = building_id;
        if (!building.roof_faces.empty() && !building.bbox.has_value()) {
            throw std::runtime_error(
                "Building roof faces were collected without a bbox");
        }
    }

    return store;
}

bool point_is_under_roof(const RoofSelectionStore &store,
                         const std::string &building_id, const Point_3 &point,
                         double vertical_buffer, double horizontal_buffer) {
    return store.contains(building_id, point, vertical_buffer,
                          horizontal_buffer);
}

bool point_is_under_roof(const RoofBuilding &building, const Point_3 &point,
                         double vertical_buffer, double horizontal_buffer) {
    return building.contains(point, vertical_buffer, horizontal_buffer);
}

void select_points_under_roofs(const std::string &input_points_file,
                               const std::string &input_roofs_file,
                               const std::string &output_points_file,
                               double vertical_buffer, double horizontal_buffer,
                               bool overwrite) {
    if (std::filesystem::exists(output_points_file) && !overwrite) {
        throw std::runtime_error("Output file already exists: " +
                                 output_points_file);
    }
    std::filesystem::create_directory(
        std::filesystem::path(output_points_file).parent_path());

    std::cout << "Reading roofs from CityJSON..." << std::endl;
    const RoofSelectionStore store = read_cityjson_roofs(input_roofs_file);
    std::cout << "Loaded buildings with roofs: " << store.buildings().size()
              << std::endl;

    std::cout << "Reading input point cloud..." << std::endl;
    NewLasReader las_reader(input_points_file);
    las_reader.points->build_kd_tree_2d();
    const std::shared_ptr<KdTree_2> kd_tree_2d =
        las_reader.points->get_kd_tree_2d();
    const auto [predefined_dims, proprietary_dims] =
        las_reader.points->dimensions();
    const std::size_t point_count =
        static_cast<std::size_t>(las_reader.points->point_count());

    NewLasWriter las_writer(predefined_dims, proprietary_dims,
                            las_reader.points->spatial_reference());

    std::cout << "Selecting points under roofs..." << std::endl;
    std::vector<bool> selected(point_count, false);

    ProgressBarTotal progress(store.buildings().size(), "Processing buildings");
    progress.start();
    for (const auto &[building_id, building] : store.buildings()) {
        (void)building_id;
        for (const auto &face : building.roof_faces) {
            if (!face.bbox.has_value()) {
                continue;
            }

            std::vector<std::size_t> candidate_indices;
            kd_tree_2d->search_indices_in_box(*face.bbox, horizontal_buffer,
                                              candidate_indices);

            for (std::size_t initial_idx : candidate_indices) {
                if (selected[initial_idx]) {
                    continue;
                }

                const Point_3 point = las_reader.points->get_point(
                    PtsStructs::PointId(initial_idx));
                if (!point_is_under_roof(store, building_id, point,
                                         vertical_buffer, horizontal_buffer)) {
                    continue;
                }

                const PtsStructs::PointId output_idx(
                    las_writer.points->point_count());
                for (pdal::Dimension::Id dim : predefined_dims) {
                    las_writer.points->copy_field<double>(
                        dim, output_idx, las_reader.points,
                        PtsStructs::PointId(initial_idx));
                }
                for (const auto &dim : proprietary_dims) {
                    las_writer.points->copy_field<double>(
                        dim, output_idx, las_reader.points,
                        PtsStructs::PointId(initial_idx));
                }

                selected[initial_idx] = true;
            }
        }
        progress.increment(1);
    }
    progress.finish();

    const std::size_t selected_count =
        std::count(selected.begin(), selected.end(), true);
    std::cout << "Writing selected points..." << std::endl;
    las_writer.write(output_points_file, {});
    std::cout << "Selection finished. Selected " << selected_count << " / "
              << point_count << " points." << std::endl;
}

} // namespace PointSelection
