#include "footprints_new.hpp"

#include <filesystem>
#include <iomanip>

#include "../edge_matching/topology.hpp"
#include "../las/reader.hpp"
#include "../parquet/reader.hpp"
#include "../utils/string_helper.hpp"
#include "constants.hpp"
#include "criterion.hpp"
#include "points_selection.hpp"

namespace EdgeMatching {

std::unique_ptr<ICriterion> AllFootprints::create_criterion(
    std::vector<Point_2> points, std::vector<double> weights,
    std::vector<Vector_2> point_inward_dirs,
    std::vector<LASclassification::Value> point_classes) const {
    return std::make_unique<CriterionFootprints>(
        std::move(points), std::move(weights), std::move(point_classes));
}

void AllFootprints::compute_weights(
    PtsStructs::StoragePtr las_points,
    const std::vector<PtsStructs::PointId> &point_ids,
    std::vector<double> &weights,
    std::vector<Vector_2> &point_inward_dirs) const {
    weights.clear();
    weights.resize(point_ids.size());
    point_inward_dirs.clear();
    point_inward_dirs.resize(point_ids.size());

    // Compute the minimum and maximum Z values
    double min_z = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    for (const auto &point_id : point_ids) {
        double z =
            las_points->get_field_as<double>(pdal::Dimension::Id::Z, point_id);
        if (z < min_z) {
            min_z = z;
        }
        if (z > max_z) {
            max_z = z;
        }
    }

    // Compute the weights for each point
    for (std::size_t i = 0; i < point_ids.size(); ++i) {
        const auto &point_id = point_ids.at(i);

        // Give more weight to higher points
        double z =
            las_points->get_field_as<double>(pdal::Dimension::Id::Z, point_id);
        double height_norm = (z - min_z + 1e-6) / (max_z - min_z + 1e-6);
        // double height_factor = height_norm;
        double height_factor = 1.0;

        // Give more weight to non-generated points
        uint8_t is_generated = las_points->get_field_as<uint8_t>(
            CustomDimensions::Id::IsGenerated, point_id);
        double generated_factor = 0.0;
        if (is_generated == 0) {
            generated_factor = 1.0;
        } else if (is_generated == 1) {
            generated_factor = 0.6;
        } else {
            generated_factor = 0.2;
        }

        // Give more weight to points classified as building
        const auto cls_raw = las_points->get_field_as<
            std::underlying_type_t<LASclassification::Value>>(
            pdal::Dimension::Id::Classification, point_id);
        const auto cls = static_cast<LASclassification::Value>(cls_raw);
        double class_factor = 0.3;
        if (cls == LASclassification::Value::Building) {
            class_factor = 1.0;
        }

        // Combine the factors to get the final weight
        weights.at(i) = height_factor * generated_factor * class_factor;

        // Extracts the normal vector for the point
        double inward_x = las_points->get_field_as<double>(
            CustomDimensions::Id::InwardVectorX, point_id);
        double inward_y = las_points->get_field_as<double>(
            CustomDimensions::Id::InwardVectorY, point_id);
        point_inward_dirs.at(i) = Vector_2(inward_x, inward_y);
    }
}

void AllFootprints::prepare_offsets(std::vector<double> &offsets) const {
    // Define the range and precision of offsets to evaluate
    double max_offset = 1.5;
    double samples = 30;

    // Build the list of offsets to evaluate
    double precision = max_offset / samples;
    offsets.reserve(2 * samples);
    offsets.push_back(0.0);
    for (int sample = 1; sample <= samples; ++sample) {
        offsets.push_back(-sample * precision);
        offsets.push_back(sample * precision);
    }
}

void AllFootprints::find_relevant_points(
    const std::vector<std::map<EdgeId, double>> &configs,
    const std::set<EdgeMatching::EdgeId> edge_ids,
    const UnitVector_2 offset_direction,
    const PtsStructs::StoragePtr las_points,
    std::vector<std::size_t> &las_indices) const {
    // Compute the bounding box of all cases
    Bbox_2 all_cases_bbox;
    get_bbox_configs(configs, edge_ids, offset_direction, all_cases_bbox);
    // std::cout << "Bounding box of all configurations: " <<
    // std::setprecision(10)
    //           << all_cases_bbox << std::endl;

    // Select all the necessary LAS points for the metric computation
    std::vector<std::size_t> las_indices_in_bbox;
    las_points->get_kd_tree_2d()->search_indices_in_box(all_cases_bbox, 0.0,
                                                        las_indices_in_bbox);

    // std::cout << "Found " << las_indices_in_bbox.size()
    //           << " LAS points in the bounding box of all configurations"
    //           << std::endl;

    // Get the roof faces corresponding to the edges
    std::vector<std::reference_wrapper<const PointSelection::RoofFace>>
        roof_faces;
    std::set<OutlineId> outline_ids;
    for (const auto &edge_id : edge_ids) {
        OutlineId outline_id = get_outline_id(edge_id);
        outline_ids.insert(outline_id);
    }
    for (const auto &outline_id : outline_ids) {
        const auto &building = outline_id_to_building.at(outline_id).get();
        const auto &roof_faces_for_building = building.roof_faces;
        for (const auto &face : roof_faces_for_building) {
            roof_faces.push_back(std::cref(face));
        }
    }

    // Get only the points that are below the roof
    las_indices.clear();
    for (std::size_t idx : las_indices_in_bbox) {
        const Point_3 &point = las_points->get_point(PtsStructs::PointId(idx));
        const Point_2 point_2d(point.x(), point.y());
        const double point_z = point.z();
        for (const auto &roof_face : roof_faces) {
            std::optional<double> roof_z_at_point =
                roof_face.get().roof_height_in_or_closest(
                    point_2d, EDGE_CRITERION_DISTANCE_CLOSE);
            if (!roof_z_at_point.has_value()) {
                continue;
            }
            if (point_z < roof_z_at_point.value() - VERTICAL_MARGIN) {
                las_indices.push_back(idx);
                break;
            }
        }
    }

    // std::cout << "Found " << las_indices.size()
    //           << " LAS points that are below the roofs" << std::endl;
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
        std::cerr << "Error reading roofprints table in file '"
                  << input_roofprints_file << "': " << status.ToString()
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
    }

    std::cout << "Loaded " << roofprints.size()
              << " MultiLineStringZ roofprints" << std::endl;

    return status;
}

arrow::Status
compute_footprints(const std::string &input_roofprints_file,
                   const std::string &input_lod22_file,
                   const std::string &input_points_file,
                   const std::string &output_footprints_template_file,
                   uint iterations, bool overwrite) {
    arrow::Status status;

    // Check if any of the outputs already exist and throw an error if it is the
    // case
    for (uint i = 1; i <= iterations; ++i) {
        std::string iteration_output_file = replace_substring_first(
            output_footprints_template_file, "{iteration}", std::to_string(i));
        if (std::filesystem::exists(iteration_output_file) && !overwrite) {
            arrow::Status error_status = arrow::Status::AlreadyExists(
                "Output file already exists: " + iteration_output_file);
            std::cerr << error_status.ToString() << std::endl;
            return error_status;
        }
    }

    std::filesystem::create_directories(
        std::filesystem::path(output_footprints_template_file).parent_path());

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

    // Build the expected structure
    EdgeVector<Edge> all_edges;
    OutlineVector<OutlineAsEdges> outlines;
    OutlineVector<std::string> building_ids;
    OutlineVector<std::reference_wrapper<const PointSelection::RoofBuilding>>
        outline_id_to_building;
    std::vector<std::pair<EdgeId, EdgeId>> intersections;

    for (const auto &roofprint : roofprints) {
        std::vector<std::vector<std::vector<EdgeId>>> multi_polygons;

        // Get the building ID from the roofprint attributes
        const std::string &id = roofprint.get_id();

        // Find the corresponding building in the LoD2.2 store based on the ID
        const auto _opt_building = store.find_building(id);
        if (!_opt_building.has_value()) {
            std::cerr << "Warning: No building found in LoD2.2 store for "
                         "roofprint with id "
                      << id << ", skipping this roofprint" << std::endl;
            continue;
        }
        const PointSelection::RoofBuilding &building = _opt_building->get();

        for (int i = 0; i < roofprint.multi_polygon->getNumGeometries(); ++i) {
            std::vector<std::vector<EdgeId>> polygons;

            OGRPolygon *polygon = roofprint.multi_polygon->getGeometryRef(i);

            for (int j = -1; j < polygon->getNumInteriorRings(); ++j) {
                std::vector<EdgeId> edges;

                OGRLinearRing *ring;
                if (j == -1) {
                    ring = polygon->getExteriorRing();
                } else {
                    ring = polygon->getInteriorRing(j);
                }
                if (ring->getNumPoints() < 2) {
                    continue;
                }

                for (int k = 0; k < ring->getNumPoints() - 1; ++k) {
                    Point_3 start(ring->getX(k), ring->getY(k), ring->getZ(k));
                    Point_3 end(ring->getX(k + 1), ring->getY(k + 1),
                                ring->getZ(k + 1));
                    Edge edge(start, end, all_edges.size());

                    all_edges.emplace_back(edge);
                    edges.emplace_back(edge.get_key());
                }
                if (edges.empty()) {
                    std::cerr << "Warning: Ring with no edges in building "
                              << id << ", skipping this ring in the outline."
                              << std::endl;
                } else {
                    polygons.push_back(edges);
                }
            }
            if (polygons.empty()) {
                std::cerr << "Warning: Polygon with no rings in building " << id
                          << ", skipping this polygon in the outline."
                          << std::endl;
            } else {
                multi_polygons.push_back(polygons);
            }
        }
        if (multi_polygons.empty()) {
            std::cerr << "Warning: Roofprint with no polygons for building "
                      << id << ", skipping this roofprint." << std::endl;
            continue;
        }
        outlines.emplace_back(multi_polygons);
        building_ids.push_back(id);
        outline_id_to_building.emplace_back(std::cref(building));
    }

    // Build the AllFootprints data structure
    AllFootprints all_footprints(all_edges, outlines, intersections,
                                 outline_id_to_building);

    // Run the optimization iterations
    for (uint i = 1; i <= iterations; ++i) {
        std::cout << "Optimization iteration " << i << " / " << iterations
                  << std::endl;

        // Optimize the edge groups in the outlines
        all_footprints.optimize_all_units(las_reader.points);

        // Export the footprints for this iteration
        std::string iteration_output_file = replace_substring_first(
            output_footprints_template_file, "{iteration}", std::to_string(i));

        std::cout << "Exporting footprints for iteration " << i
                  << " to Parquet file..." << std::endl;
        status = all_footprints.export_outlines(outlines, building_ids,
                                                iteration_output_file, true);
        if (!status.ok()) {
            std::cerr << "Error exporting footprints for iteration " << i
                      << ": " << status.ToString() << std::endl;
            return status;
        }
    }

    return status;
}

} // namespace EdgeMatching