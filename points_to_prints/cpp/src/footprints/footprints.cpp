#include "footprints.hpp"

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <ogr_geometry.h>
#include <optional>

#include "../las/reader.hpp"
#include "../las/writer.hpp"
#include "../parquet.hpp"
#include "../parquet/reader.hpp"
#include "../points.hpp"
#include "../utils/cgal.hpp"
#include "../utils/pbar.hpp"
#include "points_selection.hpp"
#include "simple_scorer.hpp"

/**
 * A vertical plane in 3D space, defined by a segment in the XY plane.
 * In its local space, the X axis is along the segment, the Y axis is vertical,
 * and the Z axis is normal to the plane (and therefore horizontal).
 */
struct VerticalPlane {
  protected:
    UnitVector_3 segment_direction;
    UnitVector_3 vertical_direction;
    UnitVector_3 normal_direction;
    Point_3 origin;
    double min_proj_x;
    double max_proj_x;

    double _get_projection_x(const Point_3 &point) const {
        return (point - origin) * segment_direction;
    }

    double _get_projection_y(const Point_3 &point) const {
        return (point - origin) * vertical_direction;
    }

    double _get_projection_z(const Point_3 &point) const {
        return (point - origin) * normal_direction;
    }

    Point_2 _get_projection(const Point_3 &point) const {
        return Point_2(_get_projection_x(point), _get_projection_y(point));
    }

    Point_3 _to_plane_space(const Point_3 &point) const {
        return Point_3(_get_projection_x(point), _get_projection_y(point),
                       _get_projection_z(point));
    }

    Point_3 _to_default_space(const Point_2 &point) const {
        return origin + segment_direction * point.x() +
               vertical_direction * point.y();
    }

    Point_3 _to_default_space(const Point_3 &point) const {
        return origin + segment_direction * point.x() +
               vertical_direction * point.y() + normal_direction * point.z();
    }

  public:
    VerticalPlane(const Segment_2 &segment) {
        Point_2 seg_start = segment.source();
        Point_2 seg_end = segment.target();
        UnitVector_3 horizontal_dir(seg_end.x() - seg_start.x(),
                                    seg_end.y() - seg_start.y(), 0);
        UnitVector_3 vertical_dir(0, 0, 1);
        UnitVector_3 normal_dir =
            CGAL::cross_product(horizontal_dir, vertical_dir);
        segment_direction = horizontal_dir;
        vertical_direction = vertical_dir;
        normal_direction = normal_dir;

        origin = Point_3(seg_start.x(), seg_start.y(), 0);
        min_proj_x = 0;
        max_proj_x = _get_projection_x(Point_3(seg_end.x(), seg_end.y(), 0));
    }
};

struct Vertical2DCroppedPlane : VerticalPlane {
  private:
    double min_x;
    double max_x;
    std::vector<Segment_2> limits_intervals; // in 2D in the plane, sorted by
                                             // their projection on the X axis
    double vertical_buffer;
    double min_z;
    double max_z;

    std::size_t _find_limit_interval(double x) const {
        // Binary search to find the interval in limits_intervals that contains
        // x
        std::size_t left = 0;
        std::size_t right = limits_intervals.size();
        while (left < right) {
            std::size_t mid = left + (right - left) / 2;
            if (limits_intervals[mid].source().x() <= x &&
                limits_intervals[mid].target().x() >= x) {
                return mid;
            } else if (limits_intervals[mid].source().x() > x) {
                right = mid;
            } else {
                left = mid + 1;
            }
        }
        throw std::runtime_error("No limit interval found for x = " +
                                 std::to_string(x));
    }

  public:
    Vertical2DCroppedPlane(const Segment_2 &segment,
                           const std::vector<Point_3> &limits,
                           double horizontal_buffer, double vertical_buffer,
                           double min_extrusion, double max_extrusion)
        : VerticalPlane(segment), vertical_buffer(vertical_buffer),
          min_z(-max_extrusion), max_z(-min_extrusion) {
        // The Z axis points outwards if the segments are ordered
        // counterclockwise, which is assumed here
        min_x = min_proj_x + horizontal_buffer;
        max_x = max_proj_x - horizontal_buffer;

        for (std::size_t i = 0; i < limits.size() - 1; ++i) {
            Point_2 limit_start = _get_projection(limits[i]);
            Point_2 limit_end = _get_projection(limits[i + 1]);
            if (limit_start.x() > limit_end.x()) {
                throw std::invalid_argument(
                    "Limits should be ordered in the direction of the segment");
            }
            if (limit_end.x() < min_x || limit_start.x() > max_x) {
                continue;
            }
            limits_intervals.emplace_back(limit_start, limit_end);
        }
    }

    bool is_within_limits(const Point_3 &point) const {
        const Point_3 point_plane_space = _to_plane_space(point);
        // Reject if outside of the horizontal limits
        if (point_plane_space.x() < min_x || point_plane_space.x() > max_x) {
            return false;
        }

        // Reject if outside of the extrusion limits
        if (point_plane_space.z() < min_z || point_plane_space.z() > max_z) {
            return false;
        }

        // Find the corresponding interval in limits_intervals
        std::size_t interval_idx;
        try {
            interval_idx = _find_limit_interval(point_plane_space.x());
        } catch (const std::exception &e) {
            // std::cout << "Warning: " << e.what() << " for point " << point
            //           << " in plane of origin " << origin << " and direction
            //           "
            //           << horizontal_direction << std::endl;
            return false;
        }

        // Get the height of the segment at the point's X coordinate
        const Point_2 &limit_start = limits_intervals[interval_idx].source();
        const Point_2 &limit_end = limits_intervals[interval_idx].target();
        double segment_y =
            limit_start.y() + (limit_end.y() - limit_start.y()) *
                                  (point_plane_space.x() - limit_start.x()) /
                                  (limit_end.x() - limit_start.x());
        return point_plane_space.y() <= segment_y - vertical_buffer;
    }
};

const double MIN_TRANSLATION = 0.0;
const double MAX_TRANSLATION = 1.5;
const uint TRANSLATION_STEPS = 21;
const double OPTIMIZATION_DISTANCE_THRESHOLD = 0.3;
const double OPTIMIZATION_PENALTY_DISTANCE = 1.0;

const double MIN_EXTRUSION =
    MIN_TRANSLATION - OPTIMIZATION_DISTANCE_THRESHOLD; // Pointing inwards
const double MAX_EXTRUSION =
    MAX_TRANSLATION + OPTIMIZATION_DISTANCE_THRESHOLD; // Pointing inwards
const double MIN_BUFFER = MIN_EXTRUSION;
const double MAX_BUFFER = MAX_EXTRUSION + OPTIMIZATION_PENALTY_DISTANCE;
const double HORIZONTAL_MARGIN = 0.5;
const double VERTICAL_MARGIN = 0.5;

void one_facade_to_footprint(const PtsStructs::StoragePtr &storage,
                             const std::vector<Point_3> &facade_roof_edges,
                             std::vector<std::size_t> &footprint_points,
                             Segment_2 &footprint_edge) {
    if (facade_roof_edges.size() < 2) {
        throw std::invalid_argument(
            "At least two points are required to define a facade roof edge");
    }

    // Compute the area of interest for the facade in 2D
    const Point_3 &start_3d = facade_roof_edges.front();
    const Point_3 &end_3d = facade_roof_edges.back();
    Point_2 start_2d(start_3d.x(), start_3d.y());
    Point_2 end_2d(end_3d.x(), end_3d.y());

    if (start_2d == end_2d) {
        std::cerr << "The start and end points of the facade "
                     "roof edge are the same in 2D, skipping this facade"
                  << std::endl;
        return;
    }

    UnitVector_2 direction_inwards(
        (end_2d - start_2d).perpendicular(CGAL::COUNTERCLOCKWISE));

    // double min_extrusion = MIN_EXTRUSION;
    // double max_extrusion = MAX_EXTRUSION;
    double min_buffer = MIN_BUFFER;
    double max_buffer = MAX_BUFFER;
    Segment_2 outside_boundary(start_2d + min_buffer * direction_inwards,
                               end_2d + min_buffer * direction_inwards);
    Segment_2 inside_boundary(start_2d + max_buffer * direction_inwards,
                              end_2d + max_buffer * direction_inwards);
    Bbox_2 area_of_interest = outside_boundary.bbox() + inside_boundary.bbox();

    // Prepare the vertical plane
    double horizontal_margin = HORIZONTAL_MARGIN;
    double vertical_margin = VERTICAL_MARGIN;
    Vertical2DCroppedPlane segment_2d_space(
        Segment_2(start_2d, end_2d), facade_roof_edges, horizontal_margin,
        vertical_margin, min_buffer, max_buffer);

    // Get the points in the area of interest
    auto kd_tree = storage->get_kd_tree_2d();
    std::vector<std::size_t> nearby_point_indices;
    kd_tree->search_indices_in_box(area_of_interest, 0.0, nearby_point_indices);

    // Get only the points that are below the roof edges
    footprint_points.clear();
    for (std::size_t idx : nearby_point_indices) {
        const Point_3 &point = storage->get_point(PtsStructs::PointId(idx));
        // Check if the point is within the area of interest in 2D
        Point_2 point_2d(point.x(), point.y());

        // Check if the point is within the limits defined by the facade roof
        // edges
        if (!segment_2d_space.is_within_limits(point)) {
            continue;
        }

        footprint_points.push_back(idx);
    }

    footprint_edge = Segment_2(start_2d, end_2d);
    // If no points are found, we cannot optimize the footprint edge
    if (footprint_points.empty()) {
        return;
    }

    // Prepare the translations to test for optimizing the footprint edge
    std::vector<double> translations(TRANSLATION_STEPS);
    double step_factor =
        (MAX_TRANSLATION - MIN_TRANSLATION) / (TRANSLATION_STEPS - 1);
    for (uint i = 0; i < TRANSLATION_STEPS; ++i) {
        translations[i] = MIN_TRANSLATION + step_factor * i;
    }

    // Prepare the points
    std::vector<Point_3> points;
    points.reserve(footprint_points.size());
    std::vector<LASclassification::Value> classifications;
    classifications.reserve(footprint_points.size());
    for (std::size_t idx : footprint_points) {
        PtsStructs::PointId p_id(idx);
        points.push_back(storage->get_point(p_id));
        classifications.push_back(static_cast<LASclassification::Value>(
            storage->get_field_as<uint8_t>(pdal::Dimension::Id::Classification,
                                           p_id)));
    }

    // Score the translations and keep the best one
    std::vector<double> scores;
    score_line_translations(footprint_edge.supporting_line(), points,
                            classifications, direction_inwards, translations,
                            OPTIMIZATION_DISTANCE_THRESHOLD,
                            OPTIMIZATION_PENALTY_DISTANCE, scores);
    double best_score = -std::numeric_limits<double>::infinity();
    std::size_t best_translation_idx = 0;
    for (std::size_t i = 0; i < translations.size(); ++i) {
        if (scores[i] > best_score) {
            best_score = scores[i];
            best_translation_idx = i;
        }
    }
    double best_translation = translations[best_translation_idx];
    footprint_edge = Segment_2(start_2d + best_translation * direction_inwards,
                               end_2d + best_translation * direction_inwards);
}

arrow::Status roofs_to_footprints(const std::string &input_roofprints_file,
                                  const std::string &input_points_file,
                                  const std::string &output_footprints_file,
                                  const std::string &output_points_file,
                                  bool overwrite) {
    arrow::Status status;

    if (std::filesystem::exists(output_footprints_file) && !overwrite) {
        throw std::runtime_error("Output file already exists: " +
                                 output_footprints_file);
    }

    /* ----------------------------------------------------------------------
     */
    /*                           Load the roofprints */
    /* ----------------------------------------------------------------------
     */

    // Read the roofprints data from the Parquet file using the
    // ParquetReader
    ParquetReader roofprints_reader(input_roofprints_file);

    std::shared_ptr<arrow::Table> roofprints_table;
    status = roofprints_reader.read_table(roofprints_table);
    if (!status.ok()) {
        std::cerr << "Error reading roofprints table: " << status.ToString()
                  << std::endl;
        return status;
    }

    // Check that the geometry column exists and is of the expected type
    int geometry_idx = roofprints_table->schema()->GetFieldIndex("geometry");
    if (geometry_idx < 0) {
        return arrow::Status::Invalid(
            "Column 'geometry' not found in roofprints table");
    }
    if (roofprints_table->schema()->field(geometry_idx)->type()->id() !=
        arrow::Type::BINARY) {
        return arrow::Status::Invalid(
            "Column 'geometry' is not of type Binary in roofprints table");
    }

    // Prepare the columns to read from the roofprints Parquet file
    std::vector<RequestedColumn> columns{
        {"cleabs", ParquetValueType::Utf8},
        {"origine_du_batiment", ParquetValueType::Utf8},
        {"geometry", ParquetValueType::Binary}};

    GenericParquetOutput roofprints_output;
    status = roofprints_reader.read_columns(columns, roofprints_output);
    if (!status.ok()) {
        std::cerr << "Error reading edges Parquet file: " << status.ToString()
                  << std::endl;
        return status;
    }

    // Convert the input data into the desired format
    std::vector<MultiPolygonZWithAttributes> roofprints;
    roofprints.reserve(roofprints_output.row_count);
    for (std::size_t i = 0; i < roofprints_output.row_count; ++i) {
        if (roofprints_output.value_is_null("cleabs", i) ||
            roofprints_output.value_is_null("origine_du_batiment", i) ||
            roofprints_output.value_is_null("geometry", i)) {
            continue;
        }

        std::string cleabs = roofprints_output.value<std::string>("cleabs", i);
        std::string origine_du_batiment =
            roofprints_output.value<std::string>("origine_du_batiment", i);
        OutlineSource::Id outline_source =
            OutlineSource::from_string(origine_du_batiment);

        // if (cleabs != "BATIMENT0000000337020548") {
        //     continue;
        // }

        const std::vector<uint8_t> &geometry_binary =
            roofprints_output.value<std::vector<uint8_t>>("geometry", i);

        ARROW_ASSIGN_OR_RAISE(OGRMultiPolygonPtr multi_polygon,
                              parse_wkb_multipolygonz(geometry_binary));

        roofprints.emplace_back(std::move(multi_polygon), cleabs,
                                outline_source);

        // std::cout << "Read row " << i << ": cleabs=" << cleabs
        //           << ", origine_du_batiment=" << origine_du_batiment
        //           << ", geometry="
        //           << roofprints.back().multi_polygon->exportToWkt()
        //           << std::endl;
    }

    std::cout << "Loaded " << roofprints.size()
              << " MultiLineStringZ roofprints" << std::endl;

    /* ----------------------------------------------------------------------
     */
    /*                          Load the point cloud */
    /* ----------------------------------------------------------------------
     */

    NewLasReader las_reader(input_points_file);
    auto storage = las_reader.points;
    storage->build_kd_tree_2d();
    // Trajectory trajectory = read_trajectory(trajectory_file);
    // PtsStructs::Topology3D topo(storage, trajectory);

    /* ----------------------------------------------------------------------
     */
    /*                            Process each edge */
    /* ----------------------------------------------------------------------
     */

    auto [initial_pdal_dims, initial_custom_dims] =
        las_reader.points->dimensions();
    std::vector<pdal::Dimension::Id> pdal_dims = initial_pdal_dims;
    std::vector<ProprietaryDimension> custom_dims = initial_custom_dims;
    std::vector<ProprietaryDimension> new_distances_custom_dims = {
        CustomDimensions::Id::CorrespondingBuildingId,
    };
    custom_dims.insert(custom_dims.end(), new_distances_custom_dims.begin(),
                       new_distances_custom_dims.end());
    NewLasWriter las_writer(pdal_dims, custom_dims,
                            las_reader.points->spatial_reference());
    std::vector<std::size_t> footprint_points;

    std::cout << "Processing roofprints to compute footprints..." << std::endl;
    std::vector<MultiLineStringZWithAttributes> footprints;
    footprints.reserve(roofprints.size());
    ProgressBarTotal progress_bar(roofprints.size(), "Processing roofprints");
    progress_bar.start();
    for (const auto &roofprint : roofprints) {
        OGRMultiLineString *multi_line_string_raw = new OGRMultiLineString();
        const std::string &id = roofprint.get_id();
        for (int i = 0; i < roofprint.multi_polygon->getNumGeometries(); ++i) {
            // Extract the facade roof edges as a vector of Point_3
            OGRPolygon *polygon = roofprint.multi_polygon->getGeometryRef(i);

            for (int j = 0; j < polygon->getNumInteriorRings(); ++j) {
                OGRLinearRing *ring = polygon->getInteriorRing(j);
                if (ring->getNumPoints() < 2) {
                    continue;
                }

                std::vector<Point_3> facade_roof_edges;
                for (int j = 0; j < ring->getNumPoints(); ++j) {
                    Point_3 point(ring->getX(j), ring->getY(j), ring->getZ(j));
                    facade_roof_edges.push_back(point);
                }

                // Compute the footprint edge for this facade using the point
                // cloud
                Segment_2 footprint_edge;
                one_facade_to_footprint(storage, facade_roof_edges,
                                        footprint_points, footprint_edge);

                for (const auto &point_id : footprint_points) {
                    PtsStructs::PointId new_idx(
                        las_writer.points->point_count());
                    for (const auto &dim : initial_pdal_dims) {
                        las_writer.points->copy_field<double>(
                            dim, new_idx, storage,
                            PtsStructs::PointId(point_id));
                    }
                    for (const auto &dim : initial_custom_dims) {
                        las_writer.points->copy_field<double>(
                            dim, new_idx, storage,
                            PtsStructs::PointId(point_id));
                    }
                    las_writer.points->set_field(
                        CustomDimensions::Id::CorrespondingBuildingId, new_idx,
                        building_id_to_int64(id));
                }

                // Add the computed footprint edge to the output MultiLineString
                OGRLineString *footprint_line_string = new OGRLineString();
                footprint_line_string->addPoint(footprint_edge.source().x(),
                                                footprint_edge.source().y(),
                                                0.0);
                footprint_line_string->addPoint(footprint_edge.target().x(),
                                                footprint_edge.target().y(),
                                                0.0);
                multi_line_string_raw->addGeometry(footprint_line_string);
            }
        }
        OGRMultiLineStringPtr multi_line_string(multi_line_string_raw);
        footprints.emplace_back(std::move(multi_line_string),
                                roofprint.get_id(),
                                roofprint.get_outline_source());

        progress_bar.increment(1);
    }
    progress_bar.finish();

    /* ----------------------------------------------------------------------
     */
    /*                       Write the output footprints */
    /* ----------------------------------------------------------------------
     */

    std::filesystem::create_directories(
        std::filesystem::path(output_footprints_file).parent_path());

    status = write_geoms_to_parquet(footprints, output_footprints_file, true);
    if (!status.ok()) {
        std::cerr << "Error writing footprints Parquet file: "
                  << status.ToString() << std::endl;
        return status;
    }

    /* ----------------------------------------------------------------------
     */
    /*                       Write the output points */
    /* ----------------------------------------------------------------------
     */

    std::filesystem::create_directories(
        std::filesystem::path(output_points_file).parent_path());

    las_writer.write(output_points_file, {});

    return status;
}

void one_roofprint_edge_to_footprint(
    const PtsStructs::StoragePtr &storage,
    const PointSelection::RoofBuilding &building,
    const Segment_2 &roofprint_edge, std::vector<std::size_t> &footprint_points,
    Segment_2 &footprint_edge) {
    footprint_edge = roofprint_edge;
    Point_2 start_2d = roofprint_edge.source();
    Point_2 end_2d = roofprint_edge.target();

    if (start_2d == end_2d) {
        std::cerr << std::setprecision(10) << "The start point (" << start_2d
                  << ") and end point (" << end_2d
                  << ") of the facade roof edge are the same in 2D, skipping "
                     "this facade"
                  << std::endl;
        return;
    }

    UnitVector_2 direction_inwards(
        (end_2d - start_2d).perpendicular(CGAL::COUNTERCLOCKWISE));

    // double min_extrusion = MIN_EXTRUSION;
    // double max_extrusion = MAX_EXTRUSION;
    double min_buffer = MIN_BUFFER;
    double max_buffer = MAX_BUFFER;
    Segment_2 outside_boundary(start_2d + min_buffer * direction_inwards,
                               end_2d + min_buffer * direction_inwards);
    Segment_2 inside_boundary(start_2d + max_buffer * direction_inwards,
                              end_2d + max_buffer * direction_inwards);
    Bbox_2 area_of_interest = outside_boundary.bbox() + inside_boundary.bbox();

    // Get the points in the area of interest
    auto kd_tree = storage->get_kd_tree_2d();
    std::vector<std::size_t> nearby_point_indices;
    kd_tree->search_indices_in_box(area_of_interest, 0.0, nearby_point_indices);

    // Get only the points that are below the roof edges
    std::vector<std::reference_wrapper<const PointSelection::RoofFace>>
        roof_faces;
    building.find_faces_for_segment(roofprint_edge, 1e-2, roof_faces);
    footprint_points.clear();
    for (std::size_t idx : nearby_point_indices) {
        const Point_3 &point = storage->get_point(PtsStructs::PointId(idx));
        const Point_2 point_2d(point.x(), point.y());
        const double point_z = point.z();
        for (const auto &roof_face : roof_faces) {
            std::optional<double> roof_z_at_point =
                roof_face.get().roof_height_in_or_closest(
                    point_2d, OPTIMIZATION_DISTANCE_THRESHOLD);
            if (!roof_z_at_point.has_value()) {
                continue;
            }
            if (point_z < roof_z_at_point.value() - VERTICAL_MARGIN) {
                footprint_points.push_back(idx);
                break;
            }
        }
    }

    // If no points are found, we cannot optimize the footprint edge
    if (footprint_points.empty()) {
        return;
    }

    // Prepare the translations to test for optimizing the footprint edge
    std::vector<double> translations(TRANSLATION_STEPS);
    double step_factor =
        (MAX_TRANSLATION - MIN_TRANSLATION) / (TRANSLATION_STEPS - 1);
    for (uint i = 0; i < TRANSLATION_STEPS; ++i) {
        translations[i] = MIN_TRANSLATION + step_factor * i;
    }

    // Prepare the points
    std::vector<Point_3> points;
    points.reserve(footprint_points.size());
    std::vector<LASclassification::Value> classifications;
    classifications.reserve(footprint_points.size());
    for (std::size_t idx : footprint_points) {
        PtsStructs::PointId p_id(idx);
        points.push_back(storage->get_point(p_id));
        classifications.push_back(static_cast<LASclassification::Value>(
            storage->get_field_as<uint8_t>(pdal::Dimension::Id::Classification,
                                           p_id)));
    }

    // Score the translations and keep the best one
    std::vector<double> scores;
    score_line_translations(footprint_edge.supporting_line(), points,
                            classifications, direction_inwards, translations,
                            OPTIMIZATION_DISTANCE_THRESHOLD,
                            OPTIMIZATION_PENALTY_DISTANCE, scores);
    double best_score = -std::numeric_limits<double>::infinity();
    std::size_t best_translation_idx = 0;
    for (std::size_t i = 0; i < translations.size(); ++i) {
        if (scores[i] > best_score) {
            best_score = scores[i];
            best_translation_idx = i;
        }
    }
    double best_translation = translations[best_translation_idx];
    footprint_edge = Segment_2(start_2d + best_translation * direction_inwards,
                               end_2d + best_translation * direction_inwards);
}

arrow::Status
read_roofprints(const std::string &input_roofprints_file,
                std::vector<MultiPolygonZWithAttributes> &roofprints) {
    arrow::Status status;

    // Read the roofprints data from the Parquet file using the
    // ParquetReader
    ParquetReader roofprints_reader(input_roofprints_file);

    std::shared_ptr<arrow::Table> roofprints_table;
    status = roofprints_reader.read_table(roofprints_table);
    if (!status.ok()) {
        std::cerr << "Error reading roofprints table: " << status.ToString()
                  << std::endl;
        return status;
    }

    // Check that the geometry column exists and is of the expected type
    int geometry_idx = roofprints_table->schema()->GetFieldIndex("geometry");
    if (geometry_idx < 0) {
        return arrow::Status::Invalid(
            "Column 'geometry' not found in roofprints table");
    }
    if (roofprints_table->schema()->field(geometry_idx)->type()->id() !=
        arrow::Type::BINARY) {
        return arrow::Status::Invalid(
            "Column 'geometry' is not of type Binary in roofprints table");
    }

    // Prepare the columns to read from the roofprints Parquet file
    std::vector<RequestedColumn> columns{
        {"cleabs", ParquetValueType::Utf8},
        {"origine_du_batiment", ParquetValueType::Utf8},
        {"geometry", ParquetValueType::Binary}};

    GenericParquetOutput roofprints_output;
    status = roofprints_reader.read_columns(columns, roofprints_output);
    if (!status.ok()) {
        std::cerr << "Error reading edges Parquet file: " << status.ToString()
                  << std::endl;
        return status;
    }

    // Convert the input data into the desired format
    roofprints.reserve(roofprints_output.row_count);
    for (std::size_t i = 0; i < roofprints_output.row_count; ++i) {
        if (roofprints_output.value_is_null("cleabs", i) ||
            roofprints_output.value_is_null("origine_du_batiment", i) ||
            roofprints_output.value_is_null("geometry", i)) {
            continue;
        }

        std::string cleabs = roofprints_output.value<std::string>("cleabs", i);
        std::string origine_du_batiment =
            roofprints_output.value<std::string>("origine_du_batiment", i);
        OutlineSource::Id outline_source =
            OutlineSource::from_string(origine_du_batiment);

        // if (cleabs != "BATIMENT0000000337020548") {
        //     continue;
        // }

        const std::vector<uint8_t> &geometry_binary =
            roofprints_output.value<std::vector<uint8_t>>("geometry", i);

        ARROW_ASSIGN_OR_RAISE(OGRMultiPolygonPtr multi_polygon,
                              parse_wkb_multipolygonz(geometry_binary));

        roofprints.emplace_back(std::move(multi_polygon), cleabs,
                                outline_source);

        // std::cout << "Read row " << i << ": cleabs=" << cleabs
        //           << ", origine_du_batiment=" << origine_du_batiment
        //           << ", geometry="
        //           << roofprints.back().multi_polygon->exportToWkt()
        //           << std::endl;
    }

    std::cout << "Loaded " << roofprints.size()
              << " MultiLineStringZ roofprints" << std::endl;

    return status;
}

arrow::Status roofprints_and_lod22_to_footprints(
    const std::string &input_roofprints_file,
    const std::string &input_lod22_file, const std::string &input_points_file,
    const std::string &output_footprints_file,
    const std::string &output_points_file, bool overwrite) {
    arrow::Status status;

    if (std::filesystem::exists(output_footprints_file) && !overwrite) {
        throw std::runtime_error("Output file already exists: " +
                                 output_footprints_file);
    }
    if (std::filesystem::exists(output_points_file) && !overwrite) {
        throw std::runtime_error("Output file already exists: " +
                                 output_points_file);
    }

    /* ----------------------------------------------------------------------
     */
    /*                           Load the roofprints */
    /* ----------------------------------------------------------------------
     */

    std::vector<MultiPolygonZWithAttributes> roofprints;
    status = read_roofprints(input_roofprints_file, roofprints);
    if (!status.ok()) {
        std::cerr << "Error reading roofprints: " << status.ToString()
                  << std::endl;
        return status;
    }

    /* ----------------------------------------------------------------------
     */
    /*                          Load the point cloud */
    /* ----------------------------------------------------------------------
     */

    NewLasReader las_reader(input_points_file);
    auto storage = las_reader.points;
    storage->build_kd_tree_2d();
    std::cout << "Loaded point cloud with " << storage->point_count()
              << " points" << std::endl;

    /* ----------------------------------------------------------------------
     */
    /*                        Load the LoD2.2 buildings */
    /* ----------------------------------------------------------------------
     */

    const auto store = PointSelection::read_cityjson_roofs(input_lod22_file);
    std::cout << "Loaded buildings with roofs: " << store.buildings().size()
              << std::endl;

    /* ----------------------------------------------------------------------
     */
    /*                            Process each edge */
    /* ----------------------------------------------------------------------
     */

    auto [initial_pdal_dims, initial_custom_dims] =
        las_reader.points->dimensions();
    std::vector<pdal::Dimension::Id> pdal_dims = initial_pdal_dims;
    std::vector<ProprietaryDimension> custom_dims = initial_custom_dims;
    std::vector<ProprietaryDimension> new_distances_custom_dims = {
        CustomDimensions::Id::CorrespondingBuildingId,
        CustomDimensions::Id::CorrespondingEdgeId,
    };
    custom_dims.insert(custom_dims.end(), new_distances_custom_dims.begin(),
                       new_distances_custom_dims.end());
    NewLasWriter las_writer(pdal_dims, custom_dims,
                            las_reader.points->spatial_reference());
    std::vector<std::size_t> footprint_points;

    std::cout << "Processing roofprints to compute footprints..." << std::endl;
    std::vector<MultiLineStringZWithAttributes> footprints;
    footprints.reserve(roofprints.size());
    ProgressBarTotal progress_bar(roofprints.size(), "Processing roofprints");
    progress_bar.start();
    for (const auto &roofprint : roofprints) {
        OGRMultiLineString *multi_line_string_raw = new OGRMultiLineString();
        const std::string &id = roofprint.get_id();
        const std::optional<PointSelection::RoofBuilding> _opt_building =
            store.find_building(id);
        if (!_opt_building.has_value()) {
            std::cerr << "Warning: No building found in LoD2.2 store for "
                         "roofprint with id "
                      << id << ", skipping this roofprint" << std::endl;
            continue;
        }
        const PointSelection::RoofBuilding &building = _opt_building.value();

        for (int i = 0; i < roofprint.multi_polygon->getNumGeometries(); ++i) {
            // Extract the facade roof edges as a vector of Point_3
            OGRPolygon *polygon = roofprint.multi_polygon->getGeometryRef(i);

            for (int j = -1; j < polygon->getNumInteriorRings(); ++j) {
                OGRLinearRing *ring;
                if (j == -1) {
                    ring = polygon->getExteriorRing();
                } else {
                    ring = polygon->getInteriorRing(j);
                }
                if (ring->getNumPoints() < 2) {
                    continue;
                }

                // Iterate over the points of the ring
                for (int k = 0; k < ring->getNumPoints() - 1; ++k) {
                    Point_2 start(ring->getX(k), ring->getY(k));
                    Point_2 end(ring->getX(k + 1), ring->getY(k + 1));
                    Segment_2 roofprint_edge(start, end);

                    // Compute the footprint edge for this facade using the
                    // point cloud
                    Segment_2 footprint_edge;
                    one_roofprint_edge_to_footprint(
                        storage, building, roofprint_edge, footprint_points,
                        footprint_edge);

                    // Add the computed footprint edge to the facade roof edge
                    // line string
                    OGRLineString *facade_roof_edge_line_string =
                        new OGRLineString();
                    facade_roof_edge_line_string->addPoint(
                        footprint_edge.source().x(),
                        footprint_edge.source().y(), 0.0);
                    facade_roof_edge_line_string->addPoint(
                        footprint_edge.target().x(),
                        footprint_edge.target().y(), 0.0);
                    multi_line_string_raw->addGeometry(
                        facade_roof_edge_line_string);

                    // Write the points corresponding to the computed footprint
                    // edge to the output LAS file, with the corresponding
                    // building and edge IDs in custom dimensions
                    for (const auto &point_id : footprint_points) {
                        PtsStructs::PointId new_idx(
                            las_writer.points->point_count());
                        for (const auto &dim : initial_pdal_dims) {
                            las_writer.points->copy_field<double>(
                                dim, new_idx, storage,
                                PtsStructs::PointId(point_id));
                        }
                        for (const auto &dim : initial_custom_dims) {
                            las_writer.points->copy_field<double>(
                                dim, new_idx, storage,
                                PtsStructs::PointId(point_id));
                        }
                        las_writer.points->set_field(
                            CustomDimensions::Id::CorrespondingBuildingId,
                            new_idx, building_id_to_int64(id));
                        las_writer.points->set_field(
                            CustomDimensions::Id::CorrespondingEdgeId, new_idx,
                            k);
                    }
                }
            }
        }
        OGRMultiLineStringPtr multi_line_string(multi_line_string_raw);
        footprints.emplace_back(std::move(multi_line_string),
                                roofprint.get_id(),
                                roofprint.get_outline_source());

        progress_bar.increment(1);
    }
    progress_bar.finish();

    /* ----------------------------------------------------------------------
     */
    /*                       Write the output footprints */
    /* ----------------------------------------------------------------------
     */

    std::filesystem::create_directories(
        std::filesystem::path(output_footprints_file).parent_path());

    status = write_geoms_to_parquet(footprints, output_footprints_file, true);
    if (!status.ok()) {
        std::cerr << "Error writing footprints Parquet file: "
                  << status.ToString() << std::endl;
        return status;
    }

    /* ----------------------------------------------------------------------
     */
    /*                       Write the output points */
    /* ----------------------------------------------------------------------
     */

    std::filesystem::create_directories(
        std::filesystem::path(output_points_file).parent_path());

    las_writer.write(output_points_file, {});

    return status;
}