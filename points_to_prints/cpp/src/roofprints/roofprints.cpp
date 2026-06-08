#include "roofprints.hpp"

#include <filesystem>

#include "../edge_matching/topology.hpp"
#include "../las/reader.hpp"
#include "../parquet/reader.hpp"
#include "../utils/string_helper.hpp"
#include "criterion.hpp"

namespace EdgeMatching {

std::unique_ptr<ICriterion> AllRoofprints::create_criterion(
    std::vector<Point_2> points, std::vector<double> weights,
    std::vector<Vector_2> point_inward_dirs,
    std::vector<LASclassification::Value> point_classes) const {
    return std::make_unique<CriterionRoofprints>(
        std::move(points), std::move(weights), std::move(point_inward_dirs));
}

void AllRoofprints::compute_weights(
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

void AllRoofprints::prepare_offsets(std::vector<double> &offsets) const {
    // Define the range and precision of offsets to evaluate
    double max_offset = 2.0;
    uint samples_one_side = 40;

    // Build the list of offsets to evaluate
    double precision = (2 * max_offset) / (2 * samples_one_side);
    offsets.reserve(2 * samples_one_side + 1);
    offsets.push_back(0.0); // Include zero offset
    for (int sample = 1; sample <= samples_one_side; ++sample) {
        offsets.push_back(-sample * precision);
        offsets.push_back(sample * precision);
    }
}

void AllRoofprints::find_relevant_points(
    const std::vector<std::map<EdgeId, double>> &configs,
    const std::set<EdgeMatching::EdgeId> edge_ids,
    const UnitVector_2 offset_direction,
    const PtsStructs::StoragePtr las_points,
    std::vector<std::size_t> &las_indices) const {
    // Compute the bounding box of all cases
    Bbox_2 all_cases_bbox;
    get_bbox_configs(configs, edge_ids, offset_direction, all_cases_bbox);

    // Select all the necessary LAS points for the metric computation
    las_points->get_kd_tree_2d()->search_indices_in_box(all_cases_bbox, 0.0,
                                                        las_indices);
}

arrow::Status
compute_roofprints(const std::string &input_las_file,
                   const std::string &input_bd_topo_edges_file,
                   const std::string &input_bd_topo_intersections_file,
                   const std::string &output_roofprints_template_file,
                   uint iterations, bool overwrite) {
    arrow::Status status;

    // Check if any of the outputs already exist and throw an error if it is the
    // case
    for (uint i = 1; i <= iterations; ++i) {
        std::string iteration_output_file = replace_substring_first(
            output_roofprints_template_file, "{iteration}", std::to_string(i));
        if (std::filesystem::exists(iteration_output_file) && !overwrite) {
            arrow::Status error_status = arrow::Status::AlreadyExists(
                "Output file already exists: " + iteration_output_file);
            std::cerr << error_status.ToString() << std::endl;
            return error_status;
        }
    }

    std::filesystem::create_directories(
        std::filesystem::path(output_roofprints_template_file).parent_path());

    // Read the LAS file and get the point view
    std::cout << "Reading LAS file..." << std::endl;
    LasReader las_reader(input_las_file);
    std::cout << "Successfully read LAS file with "
              << las_reader.points->point_count() << " points." << std::endl;

    // Build the 2D KD-tree for the LAS points to enable efficient spatial
    // queries
    std::cout << "Building 2D KD-tree for LAS points..." << std::endl;
    las_reader.points->build_kd_tree_2d();

    // Read the building outlines from the BD TOPO file
    std::cout << "Reading building outlines from BD TOPO files..." << std::endl;
    std::vector<BDTOPOEdge> initial_edges;
    std::vector<std::pair<uint32_t, uint32_t>> _intersections;
    status = read_bd_topo_as_grouped_edges(input_bd_topo_edges_file,
                                           input_bd_topo_intersections_file,
                                           initial_edges, _intersections);
    if (!status.ok()) {
        std::cerr << "Error reading BD TOPO: " << status.ToString()
                  << std::endl;
        return status;
    }
    std::cout << "Successfully read " << initial_edges.size()
              << " edges from BD TOPO." << std::endl;

    // Format the edges for the AllOutlines data structures
    EdgeVector<Edge> edges;
    std::map<uint32_t, EdgeId> edge_key_map;
    for (const auto &edge : initial_edges) {
        edges.push_back(Edge(edge.start, edge.end, edge.edge_key));
        edge_key_map[edge.edge_key] = EdgeId(edges.size() - 1);
        // std::cout << "Mapped edge key " << edge.edge_key << " to edge id "
        //           << edge_key_map[edge.edge_key] << std::endl;
    }
    std::vector<std::pair<EdgeId, EdgeId>> intersections;
    for (const auto &intersection : _intersections) {
        EdgeId edge_id_1 = EdgeId(intersection.first);
        EdgeId edge_id_2 = EdgeId(intersection.second);
        intersections.push_back({edge_id_1, edge_id_2});
        // uint32_t edge_key_1 = intersection.first;
        // uint32_t edge_key_2 = intersection.second;
        // std::cout << "Mapping intersection between edge keys " << edge_key_1
        //           << " and " << edge_key_2 << std::endl;
        // intersections.push_back(
        //     {edge_key_map.at(edge_key_1), edge_key_map.at(edge_key_2)});
    }

    // Rebuild the MultiPolygon hierarchy based on the edges and their
    // building, polygon, and ring indices
    std::map<std::string, std::vector<std::vector<std::vector<EdgeId>>>>
        building_id_to_multi_polygons;
    std::map<std::string, std::vector<std::vector<std::vector<bool>>>>
        found_edges;
    for (const auto &edge : initial_edges) {
        auto building_id = edge.building_id;
        auto polygon_idx = edge.polygon_idx;
        auto ring_idx = edge.ring_idx;
        auto edge_idx = edge.edge_idx;
        auto edge_key = edge.edge_key;

        if (building_id_to_multi_polygons[building_id].size() <= polygon_idx) {
            building_id_to_multi_polygons[building_id].resize(polygon_idx + 1);
            found_edges[building_id].resize(polygon_idx + 1);
        }
        if (building_id_to_multi_polygons[building_id][polygon_idx].size() <=
            ring_idx) {
            building_id_to_multi_polygons[building_id][polygon_idx].resize(
                ring_idx + 1);
            found_edges[building_id][polygon_idx].resize(ring_idx + 1);
        }
        if (building_id_to_multi_polygons[building_id][polygon_idx][ring_idx]
                .size() <= edge_idx) {
            building_id_to_multi_polygons[building_id][polygon_idx][ring_idx]
                .resize(edge_idx + 1);
            found_edges[building_id][polygon_idx][ring_idx].resize(edge_idx + 1,
                                                                   false);
        }
        building_id_to_multi_polygons[building_id][polygon_idx][ring_idx]
                                     [edge_idx] = edge_key_map.at(edge_key);
        found_edges[building_id][polygon_idx][ring_idx][edge_idx] = true;
    }

    // Check for any missing edges in the hierarchy and throw an error if
    // any are found
    for (const auto &[building_id, multi_polygons] :
         building_id_to_multi_polygons) {
        for (std::size_t polygon_idx = 0; polygon_idx < multi_polygons.size();
             ++polygon_idx) {
            for (std::size_t ring_idx = 0;
                 ring_idx < multi_polygons[polygon_idx].size(); ++ring_idx) {
                for (std::size_t edge_idx = 0;
                     edge_idx < multi_polygons[polygon_idx][ring_idx].size();
                     ++edge_idx) {
                    if (!found_edges[building_id][polygon_idx][ring_idx]
                                    [edge_idx]) {
                        throw std::runtime_error(
                            "Warning: Missing edge for building " +
                            building_id + ", polygon " +
                            std::to_string(polygon_idx) + ", ring " +
                            std::to_string(ring_idx));
                    }
                }
            }
        }
    }

    // Build the expected structure
    OutlineVector<OutlineAsEdges> outlines;
    OutlineVector<std::string> building_ids;
    for (const auto &[building_id, multi_polygons] :
         building_id_to_multi_polygons) {
        std::vector<std::vector<std::vector<EdgeId>>> outline_multi_polygons;
        for (const auto &multi_polygon : multi_polygons) {
            std::vector<std::vector<EdgeId>> outline_polygons;
            for (const auto &polygon : multi_polygon) {
                std::vector<EdgeId> outline_edges;
                for (const auto &edge_id : polygon) {
                    outline_edges.push_back(edge_id);
                }
                if (!outline_edges.empty()) {
                    outline_polygons.push_back(outline_edges);
                } else {
                    std::cerr << "Warning: Polygon with no edges in building "
                              << building_id
                              << ", skipping this polygon in the outline."
                              << std::endl;
                }
            }
            if (!outline_polygons.empty()) {
                outline_multi_polygons.push_back(outline_polygons);
            } else {
                std::cerr
                    << "Warning: MultiPolygon with no polygons in building "
                    << building_id
                    << ", skipping this multi-polygon in the outline."
                    << std::endl;
            }
        }
        if (!outline_multi_polygons.empty()) {
            OutlineAsEdges outline_as_edges(outline_multi_polygons);
            outlines.push_back(outline_as_edges);
            building_ids.push_back(building_id);
        } else {
            std::cerr << "Warning: Building " << building_id
                      << " has no valid polygons, skipping this building in "
                         "the outlines."
                      << std::endl;
        }
    }

    // Build the AllOutlines data structure
    AllRoofprints all_roofprints(edges, outlines, intersections);

    // Run the optimization iterations
    for (uint i = 1; i <= iterations; ++i) {
        std::cout << "Optimization iteration " << i << " / " << iterations
                  << std::endl;

        // Optimize the edge groups in the outlines
        all_roofprints.optimize_all_units(las_reader.points);

        // Export the footprints for this iteration
        std::string iteration_output_file = replace_substring_first(
            output_roofprints_template_file, "{iteration}", std::to_string(i));

        std::cout << "Exporting roofprints for iteration " << i
                  << " to Parquet file..." << std::endl;
        status = all_roofprints.export_outlines(outlines, building_ids,
                                                iteration_output_file, true);
        if (!status.ok()) {
            std::cerr << "Error exporting roofprints for iteration " << i
                      << ": " << status.ToString() << std::endl;
            return status;
        }
    }

    return status;
}

} // namespace EdgeMatching