// #include "roofprints_new.hpp"

// #include <CGAL/aff_transformation_tags.h>
// #include <cstddef>
// #include <iomanip>
// #include <vector>

// #include "constants.hpp"
// #include "utils/ogc_simple_features.hpp"
// #include "kd_tree.hpp"
// #include "las/reader.hpp"
// #include "parquet.hpp"
// #include "points.hpp"
// #include "utils/cgal.hpp"

// // TODO: Use lines to store the geometry of the edges instead of points.
// // This means that each line will need to know its neighbours to know its
// start
// // and end.
// // This will make it easier to keep parallelism and remember the direction of
// // the lines even if they get reduced to a point.

// namespace {

// //
// -----------------------------------------------------------------------------
// // Internal geometric helpers
// //
// -----------------------------------------------------------------------------

// CustomCGAL::Angle get_smallest_angle_between(const Segment_2 &edge1,
//                                              const Segment_2 &edge2) {
//     // Compute the smallest geometric angle between the two edge directions.
//     Vector_2 dir1 = edge1.to_vector();
//     Vector_2 dir2 = edge2.to_vector();
//     double angle = CustomCGAL::angle(dir1, dir2).in_degrees();
//     if (angle > 180.0) {
//         angle = 360.0 - angle;
//     }
//     if (angle > 90.0) {
//         angle = 180.0 - angle;
//     }
//     return CustomCGAL::Angle::from_degrees(angle);
// }

// bool should_merge_edge_sequences(const Point_2 &p0, const Point_2 &p1,
//                                  const Point_2 &p2) {
//     // Reject if the two consecutive edges are not close enough to collinear
//     // according to the angular threshold.
//     double angle = CustomCGAL::angle(p0, p1, p2).in_180().in_degrees();
//     if (angle < GROUPING_ANGLE_THRESHOLD_DEGREES) {
//         return false;
//     }

//     return true;

//     // // Form the candidate merged segment and guard against a degenerate
//     // // zero-length merge.
//     // Segment_2 merged_segment(p0, p2);
//     // const double merged_segment_length =
//     //     std::sqrt(merged_segment.squared_length());
//     // if (merged_segment_length == 0.0) {
//     //     return false;
//     // }

//     // // Project the shared vertex onto the merged support line and accept
//     the
//     // // merge only if the projection error is small enough.
//     // Line_2 merged_line(p0, p2);
//     // Point_2 projection = merged_line.projection(p1);
//     // const double distance_to_projection =
//     //     std::sqrt(CGAL::squared_distance(projection, p1));

//     // return distance_to_projection <
//     //        MAX_PROJECTION_DISTANCE_FACTOR * merged_segment_length;
// }

// void compute_weights(PtsStructs::StoragePtr las_points,
//                      const std::vector<PtsStructs::PointId> &point_ids,
//                      std::vector<double> &weights) {
//     weights.clear();
//     weights.reserve(point_ids.size());

//     // Compute the minimum and maximum Z values
//     double min_z = std::numeric_limits<double>::infinity();
//     double max_z = -std::numeric_limits<double>::infinity();
//     for (const auto &point_id : point_ids) {
//         double z =
//             las_points->get_field_as<double>(pdal::Dimension::Id::Z,
//             point_id);
//         if (z < min_z) {
//             min_z = z;
//         }
//         if (z > max_z) {
//             max_z = z;
//         }
//     }

//     // Compute the weights for each point
//     for (std::size_t i = 0; i < point_ids.size(); ++i) {
//         const auto &point_id = point_ids[i];

//         // Give more weight to higher points
//         double z =
//             las_points->get_field_as<double>(pdal::Dimension::Id::Z,
//             point_id);
//         double height_norm = (z - min_z + 1e-6) / (max_z - min_z + 1e-6);
//         double height_factor = 1.0 + 2.0 * height_norm;

//         // Give more weight to non-generated points
//         uint8_t is_generated = las_points->get_field_as<uint8_t>(
//             CustomDimensions::Id::IsGenerated, point_id);
//         double generated_factor = 3 - is_generated;

//         // Give more weight to points classified as building
//         const auto cls_raw = las_points->get_field_as<
//             std::underlying_type_t<LASclassification::Value>>(
//             pdal::Dimension::Id::Classification, point_id);
//         const auto cls = static_cast<LASclassification::Value>(cls_raw);
//         double class_factor = 1.0;
//         if (cls == LASclassification::Value::Building) {
//             class_factor = 2.0;
//         }

//         // Combine the factors to get the final weight
//         weights[i] = height_factor * generated_factor * class_factor;
//     }
// }

// } // namespace

// //
// =============================================================================
// // Roofprints::AllPoints
// //
// =============================================================================

// Roofprints::AllPoints::AllPoints(const std::vector<Point_2> &points)
//     : points(points) {}

// Point_2 Roofprints::AllPoints::get(PointId point_id) const {
//     if (point_id < 0 || point_id >= points.size()) {
//         throw std::out_of_range("Point ID out of range: " +
//                                 std::to_string(point_id));
//     }
//     // std::cout << "Getting point with ID: " << point_id << std::endl;
//     // std::cout << "Point coordinates: " << points[point_id] << std::endl;
//     return points[point_id];
// }

// void Roofprints::AllPoints::set(PointId point_id, Point_2 point) {
//     if (point_id < 0 || point_id >= points.size()) {
//         throw std::out_of_range("Point ID out of range: " +
//                                 std::to_string(point_id));
//     }
//     points[point_id] = point;
// }

// //
// =============================================================================
// // Roofprints::MovableLine
// //
// =============================================================================

// Roofprints::MovableLine::MovableLine(Point_2 initial_start, Line_2 line)
//     : initial_start(initial_start), line(line), direction(line.to_vector()) {
//     double length = std::sqrt(direction.squared_length());
//     if (length > 0) {
//         direction /= length;
//     } else {
//         throw std::runtime_error("Line has zero length");
//     }
// }

// Line_2 Roofprints::MovableLine::get_line() const { return line; }
// Vector_2 Roofprints::MovableLine::get_direction() const { return direction; }

// void Roofprints::MovableLine::translate(Vector_2 offset) {
//     line = Line_2(line.point(0) + offset, line.direction());
// }
// Point_2 Roofprints::MovableLine::intersection(const Line_2 &other) const {
//     return CustomCGAL::intersection(line, other);
// }

// Point_2 Roofprints::MovableLine::intersection(const MovableLine &other) const
// {
//     return intersection(other.get_line());
// }

// //
// =============================================================================
// // Roofprints::EdgeMatching
// //
// =============================================================================

// Roofprints::EdgeMatching::EdgeMatching(const std::vector<MovableLine> &lines)
//     : lines(lines) {}

// Roofprints::MovableLine Roofprints::EdgeMatching::get(LineId line_id) const {
//     if (line_id < 0 || line_id >= lines.size()) {
//         throw std::out_of_range("Line ID out of range: " +
//                                 std::to_string(line_id));
//     }
//     return lines[line_id];
// }

// void Roofprints::EdgeMatching::set(LineId line_id, MovableLine line) {
//     if (line_id < 0 || line_id >= lines.size()) {
//         throw std::out_of_range("Line ID out of range: " +
//                                 std::to_string(line_id));
//     }
//     lines[line_id] = line;
// }

// //
// =============================================================================
// // Roofprints::EdgeSequence
// //
// =============================================================================

// Roofprints::EdgeSequence::EdgeSequence(
//     // const std::vector<PointId> &point_ids,
//     const std::vector<LineId> &line_ids)
//     : // point_ids(point_ids),
//       line_ids(line_ids) {
//     // if (point_ids.size() != line_ids.size() + 1) {
//     //     throw std::invalid_argument(
//     //         "The number of point IDs must be exactly one more than the
//     number
//     //         " "of line IDs");
//     // }
//     if (line_ids.empty()) {
//         throw std::invalid_argument(
//             "An edge sequence must have at least one line");
//     }
// }

// // Roofprints::PointId Roofprints::EdgeSequence::get_start() const {
// //     if (point_ids.empty()) {
// //         throw std::runtime_error("Edge group has no points");
// //     }
// //     return point_ids.front();
// // }

// // Roofprints::PointId Roofprints::EdgeSequence::get_end() const {
// //     if (point_ids.empty()) {
// //         throw std::runtime_error("Edge group has no points");
// //     }
// //     return point_ids.back();
// // }

// Roofprints::LineId Roofprints::EdgeSequence::get_first_line_id() const {
//     return line_ids.front();
// }

// Roofprints::LineId Roofprints::EdgeSequence::get_last_line_id() const {
//     return line_ids.back();
// }

// Point_2 Roofprints::EdgeSequence::get_first_point(Line_2 prev_line,
//                                                   EdgeMatchingPtr all_lines)
//                                                   const {
//     // The first point of the edge sequence is the intersection of the last
//     line
//     // of the previous edge sequence and the first line of the current edge
//     // sequence
//     MovableLine main_line = all_lines->get(get_last_line_id());
//     return main_line.intersection(prev_line);
// }

// Point_2 Roofprints::EdgeSequence::get_last_point(Line_2 next_line,
//                                                  EdgeMatchingPtr all_lines)
//                                                  const
//                                                  {
//     // The last point of the edge sequence is the intersection of the last
//     line
//     // of the current edge sequence and the first line of the next edge
//     // sequence
//     MovableLine main_line = all_lines->get(get_first_line_id());
//     return main_line.intersection(next_line);
// }

// Point_2 Roofprints::EdgeSequence::get_first_point(
//     Roofprints::EdgeSequence prev_edge_sequence, EdgeMatchingPtr all_lines)
//     const
//     {
//     // The first point of the edge sequence is the intersection of the last
//     line
//     // of the previous edge sequence and the first line of the current edge
//     // sequence
//     MovableLine prev_line =
//         all_lines->get(prev_edge_sequence.get_last_line_id());
//     MovableLine main_line = all_lines->get(get_first_line_id());
//     return prev_line.intersection(main_line);
// }

// Point_2 Roofprints::EdgeSequence::get_last_point(
//     Roofprints::EdgeSequence next_edge_sequence, EdgeMatchingPtr all_lines)
//     const
//     {
//     // The last point of the edge sequence is the intersection of the last
//     line
//     // of the current edge sequence and the first line of the next edge
//     // sequence
//     MovableLine main_line = all_lines->get(get_last_line_id());
//     MovableLine next_line =
//         all_lines->get(next_edge_sequence.get_first_line_id());
//     return main_line.intersection(next_line);
// }

// // void Roofprints::EdgeSequence::compute_updated_points(
// //     Point_2 new_start, Point_2 new_end, AllPointsPtr all_points,
// //     std::vector<Point_2> &new_points) const {

// //     Point_2 initial_start = all_points->get(get_start());
// //     Point_2 initial_end = all_points->get(get_end());
// //     Vector_2 initial_vec = initial_end - initial_start;

// //     Vector_2 new_vec = new_end - new_start;

// //     // std::cout << "Computing updated points for edge sequence with start
// //     // ID: "
// //     //           << start_id << " and end ID: " << end_id << std::endl;

// //     // Check if the old and the new vectors are almost parallel
// //     if (!CustomCGAL::are_almost_parallel(
// //             initial_vec, new_vec,
// //             CustomCGAL::Angle::from_degrees(
// //                 PARALLEL_ANGLE_THRESHOLD_DEGREES))) {
// //         std::cerr << "Initial start: " << std::setprecision(17) <<
// //         initial_start
// //                   << std::endl;
// //         std::cerr << "New start:     " << std::setprecision(17) <<
// new_start
// //                   << std::endl;
// //         std::cerr << "Initial end:   " << std::setprecision(17) <<
// //         initial_end
// //                   << std::endl;
// //         std::cerr << "New end:       " << std::setprecision(17) << new_end
// //                   << std::endl;
// //         std::cerr << "Initial edge vector: "
// //                   << initial_vec / std::sqrt(initial_vec.squared_length())
// //                   << std::endl;
// //         std::cerr << "New edge vector:     "
// //                   << new_vec / std::sqrt(new_vec.squared_length()) <<
// //                   std::endl;
// //         // Do not throw an error if the initial edge is smaller than 10 cm
// //         if (initial_vec.squared_length() > 1e-2) {
// //             throw std::runtime_error(
// //                 "Cannot update inner points of edge sequence because the "
// //                 "new "
// //                 "edge "
// //                 "is not in the same direction as the original edge");
// //         }
// //     }

// //     // Compute the shift and the scale factor of the movement
// //     double initial_length = std::sqrt(initial_vec.squared_length());
// //     double new_length = std::sqrt(new_vec.squared_length());
// //     if (initial_length == 0) {
// //         throw std::runtime_error("Cannot update inner points of edge "
// //                                  "sequence because the initial "
// //                                  "length of the edge is zero");
// //     }
// //     double scale_factor = new_length / initial_length;
// //     // Vector_2 shift = new_start - initial_start;

// //     // // Update the start and end points
// //     // new_points_ptr->set(start_id, new_start);
// //     // new_points_ptr->set(end_id, new_end);

// //     // // Update the inner points
// //     // for (std::size_t i = 1; i < point_ids.size() - 1; ++i) {
// //     //     PointId point_id = point_ids[i];
// //     //     Point_2 initial_point = all_points->get(point_id);
// //     //     Vector_2 initial_point_vec = initial_point - initial_start;
// //     //     Point_2 new_point =
// //     //         new_start + initial_point_vec * scale_factor + shift;
// //     //     new_points_ptr->set(point_id, new_point);
// //     // }

// //     // Get the new points in the output vector
// //     new_points.resize(point_ids.size());
// //     for (std::size_t i = 0; i < point_ids.size(); ++i) {
// //         PointId point_id = point_ids[i];
// //         Point_2 initial_point = all_points->get(point_id);
// //         Vector_2 initial_point_vec = initial_point - initial_start;
// //         Point_2 new_point = new_start + scale_factor * initial_point_vec;
// //         new_points[i] = new_point;
// //         // std::cout << "Updating point with ID: " << point_id << " from "
// //         //           << std::setprecision(17) << initial_point << " to "
// //         //           << new_point << std::endl;
// //     }

// //     // Check that the new edges are parallel to the original edges
// //     for (std::size_t i = 0; i < point_ids.size() - 1; ++i) {
// //         Point_2 initial_start = all_points->get(point_ids[i]);
// //         Point_2 initial_end = all_points->get(point_ids[i + 1]);
// //         Vector_2 initial_vec = initial_end - initial_start;

// //         Point_2 new_start = new_points[i];
// //         Point_2 new_end = new_points[i + 1];
// //         Vector_2 new_vec = new_end - new_start;

// //         if (!CustomCGAL::are_almost_parallel(
// //                 initial_vec, new_vec,
// //                 CustomCGAL::Angle::from_degrees(
// //                     PARALLEL_ANGLE_THRESHOLD_DEGREES))) {
// //             std::cerr << "Initial start: " << std::setprecision(17)
// //                       << initial_start << std::endl;
// //             std::cerr << "New start:     " << std::setprecision(17) <<
// //             new_start
// //                       << std::endl;
// //             std::cerr << "Initial end:   " << std::setprecision(17)
// //                       << initial_end << std::endl;
// //             std::cerr << "New end:       " << std::setprecision(17) <<
// //             new_end
// //                       << std::endl;
// //             std::cerr << "Initial edge vector: "
// //                       << initial_vec /
// //                       std::sqrt(initial_vec.squared_length())
// //                       << std::endl;
// //             std::cerr << "New edge vector:     "
// //                       << new_vec / std::sqrt(new_vec.squared_length())
// //                       << std::endl;
// //             throw std::runtime_error("The new edge is not in the same "
// //                                      "direction as the original edge");
// //         }
// //     }
// // }

// // void Roofprints::EdgeSequence::update_points(
// //     const std::vector<Point_2> &new_points, AllPointsPtr all_points) const
// {
// //     // Check if the number of new points matches the number of points in
// the
// //     // edge sequence
// //     if (new_points.size() != point_ids.size()) {
// //         throw std::runtime_error(
// //             "Number of new points does not match number of "
// //             "points in the edge sequence");
// //     }

// //     // Update the points in the all_points structure
// //     for (std::size_t i = 0; i < point_ids.size(); ++i) {
// //         all_points->set(point_ids[i], new_points[i]);
// //     }
// // }

// void Roofprints::EdgeSequence::compute_updated_lines(
//     Line_2 new_prev_line, Line_2 new_next_line, Vector_2 translation,
//     EdgeMatchingPtr all_lines, std::vector<MovableLine> &new_lines) const {
//     // Compute the new lines of the edge sequence based on the translation of
//     // the main edge and the positions of the neighbouring edges.

//     // Get the first and last points of the current edge sequence
//     Point_2 first_point = get_first_point(new_prev_line, all_lines);
//     Point_2 last_point = get_last_point(new_next_line, all_lines);

//     // Compute the new lines by translating the original lines
//     new_lines.resize(line_ids.size());
//     for (std::size_t i = 0; i < line_ids.size(); ++i) {
//         MovableLine original_line = all_lines->get(line_ids[i]);
//         MovableLine new_line = original_line;
//         new_line.translate(translation);
//         new_lines[i] = new_line;
//     }
// }

// //
// =============================================================================
// // Roofprints::EdgeSequenceWithNeighbours
// //
// =============================================================================

// Roofprints::EdgeSequenceWithNeighbours::EdgeSequenceWithNeighbours(
//     const EdgeSequence &main, const EdgeSequence &neighbour_1,
//     const EdgeSequence &neighbour_2)
//     : main(main) {
//     auto start_main = main.get_start();
//     auto end_main = main.get_end();
//     auto start_1 = neighbour_1.get_start();
//     auto end_1 = neighbour_1.get_end();
//     auto start_2 = neighbour_2.get_start();
//     auto end_2 = neighbour_2.get_end();

//     // Find how they are connected
//     if (end_1 == start_main && end_main == start_2) {
//         prev = neighbour_1;
//         next = neighbour_2;
//     } else if (end_2 == start_main && end_main == start_1) {
//         prev = neighbour_2;
//         next = neighbour_1;
//     } else {
//         throw std::runtime_error("The provided edge sequences are not "
//                                  "connected in the correct way");
//     }

//     // Check for cyclic connections
//     if (main.get_end() == prev.get_start()) {
//         throw std::runtime_error(
//             "Previous and main edge sequences are cyclically connected");
//     }
//     if (main.get_start() == next.get_end()) {
//         throw std::runtime_error(
//             "Next and main edge sequences are cyclically connected");
//     }
// }

// std::tuple<double, double>
// Roofprints::EdgeSequenceWithNeighbours::compute_movement_limits(
//     Vector_2 moving_direction, AllPointsPtr all_points) const {
//     // Compute the maximum and minimum offset for the moving direction.
//     // The constraint for keeping the geometry valid is that we can move the
//     // main edges in a direction as long as none of the edges is reduced to
//     // a point.

//     // Check if the moving_direction is normalized
//     if (std::abs(std::sqrt(moving_direction.squared_length()) - 1.0) > 1e-6)
//     {
//         std::cerr << "Moving direction: " << moving_direction << std::endl;
//         std::cerr << "Moving direction length: "
//                   << std::sqrt(moving_direction.squared_length()) <<
//                   std::endl;
//         throw std::runtime_error("Moving direction is not normalized");
//     }

//     Point_2 prev_start = all_points->get(prev.get_start());
//     Point_2 main_start = all_points->get(main.get_start());
//     Point_2 main_end = all_points->get(main.get_end());
//     Point_2 next_end = all_points->get(next.get_end());

//     // Check if the consecutive edges are almost collinear
//     const auto angle_prev = CustomCGAL::angle(prev_start, main_start,
//     main_end)
//                                 .in_180()
//                                 .in_degrees();
//     const auto angle_next =
//         CustomCGAL::angle(main_start, main_end,
//         next_end).in_180().in_degrees();
//     const double low_threshold_degrees = 0.1;
//     const double high_threshold_degrees = 180.0 - low_threshold_degrees;
//     if (angle_prev < low_threshold_degrees ||
//         angle_prev > high_threshold_degrees) {
//         std::cerr << "Previous edge: " << std::setprecision(17) << prev_start
//                   << " -> " << main_start << std::endl;
//         std::cerr << "Main edge: " << std::setprecision(17) << main_start
//                   << " -> " << main_end << std::endl;
//         throw std::runtime_error("Previous edge sequence is almost collinear
//         "
//                                  "with the main edge sequence");
//     }
//     if (angle_next < low_threshold_degrees ||
//         angle_next > high_threshold_degrees) {
//         std::cerr << "Main start index: " << main.get_start() << std::endl;
//         std::cerr << "Main end index:   " << main.get_end() << std::endl;
//         std::cerr << "Next end index:   " << next.get_end() << std::endl;
//         std::cerr << "Main edge: " << std::setprecision(17) << main_start
//                   << " -> " << main_end << std::endl;
//         std::cerr << "Next edge: " << std::setprecision(17) << main_end
//                   << " -> " << next_end << std::endl;
//         throw std::runtime_error("Next edge sequence is almost collinear with
//         "
//                                  "the main edge sequence");
//     }

//     double min_limit = -std::numeric_limits<double>::infinity();
//     double max_limit = std::numeric_limits<double>::infinity();

//     // First constraint: the previous edge should not be reduced to a point
//     Vector_2 previous_edge_vec = prev_start - main_start;
//     double previous_edge_on_moving_dir = previous_edge_vec *
//     moving_direction; if (previous_edge_on_moving_dir > 0) {
//         max_limit = std::min(max_limit, previous_edge_on_moving_dir);
//     } else {
//         min_limit = std::max(min_limit, previous_edge_on_moving_dir);
//     }

//     // Second constraint: the next edge should not be reduced to a point
//     Vector_2 next_edge_vec = next_end - main_end;
//     double next_edge_on_moving_dir = next_edge_vec * moving_direction;
//     if (next_edge_on_moving_dir > 0) {
//         max_limit = std::min(max_limit, next_edge_on_moving_dir);
//     } else {
//         min_limit = std::max(min_limit, next_edge_on_moving_dir);
//     }

//     // Third constraint: the main edge should not be reduced to a point
//     // Find the intersection point of the lines of the previous and next
//     // edges
//     Line_2 previous_line(prev_start, main_start);
//     Line_2 next_line(main_end, next_end);
//     auto intersection = CGAL::intersection(previous_line, next_line);
//     if (intersection) {
//         if (const Line_2 *intersection_line =
//                 std::get_if<Line_2>(&*intersection)) {
//             throw std::runtime_error(
//                 "Previous and next edge sequences are collinear");
//         } else if (const Point_2 *intersection_point =
//                        std::get_if<Point_2>(&*intersection)) {
//             Vector_2 start_to_intersection_vec =
//                 *intersection_point - main_start;
//             double start_to_intersection_on_moving_dir =
//                 start_to_intersection_vec * moving_direction;
//             if (start_to_intersection_on_moving_dir > 0) {
//                 max_limit =
//                     std::min(max_limit, start_to_intersection_on_moving_dir);
//             } else {
//                 min_limit =
//                     std::max(min_limit, start_to_intersection_on_moving_dir);
//             }
//         } else {
//             throw std::runtime_error("Unexpected intersection type");
//         }
//     }

//     return {min_limit, max_limit};
// }

// double compute_metric(Point_2 las_point, Roofprints::Edge edge) {
//     // Check if the point projected on the edge is within the edge
//     // segment
//     Vector_2 start_to_point = las_point - edge.get_start();
//     double projection_length = start_to_point * edge.get_direction();
//     if (projection_length < 0 || projection_length > edge.get_length()) {
//         return 0.0;
//     }

//     // Compute the distance from the point to the edge and the score
//     // based on the distance
//     double dist =
//         std::sqrt(CGAL::squared_distance(edge.to_segment(), las_point));
//     double score;
//     if (dist > METRIC_DISTANCE_0) {
//         score = 0.0;
//     } else if (dist < METRIC_DISTANCE_1) {
//         score = 1.0;
//     } else {
//         score = 1.0 - (dist - METRIC_DISTANCE_1) /
//                           (METRIC_DISTANCE_0 - METRIC_DISTANCE_1);
//     }
//     return score;
// }

// //
// -----------------------------------------------------------------------------
// // Internal metric helpers
// //
// -----------------------------------------------------------------------------

// void Roofprints::EdgeSequenceWithNeighbours::compute_metric_for_offsets(
//     const Vector_2 &moving_direction, const std::vector<double> &offsets,
//     AllPointsPtr points_outlines, const KdTree_2 &las_kd_tree,
//     PtsStructs::StoragePtr las_points, std::vector<double> &metrics) const {
//     // TODO: Compute the metric efficiently for all offsets by projecting
//     // only once and then checking whether the projected points are within
//     // the edge segment for each offset

//     // Extract the relevant points and directions
//     Point_2 prev_start = points_outlines->get(prev.get_start());
//     Point_2 main_start = points_outlines->get(main.get_start());
//     Point_2 main_end = points_outlines->get(main.get_end());
//     Point_2 next_end = points_outlines->get(next.get_end());
//     Vector_2 prev_direction = main_start - prev_start;
//     prev_direction =
//         prev_direction / std::sqrt(prev_direction.squared_length());
//     Vector_2 next_direction = next_end - main_end;
//     next_direction =
//         next_direction / std::sqrt(next_direction.squared_length());

//     double prev_scale =
//         1 /
//         std::cos(
//             CustomCGAL::angle(prev_direction,
//             moving_direction).in_radians());
//     double next_scale =
//         1 /
//         std::cos(
//             CustomCGAL::angle(next_direction,
//             moving_direction).in_radians());

//     // Extract the points from the point cloud
//     double min_offset = *std::min_element(offsets.begin(), offsets.end());
//     double max_offset = *std::max_element(offsets.begin(), offsets.end());
//     double max_absolute_offset =
//         std::max(std::abs(min_offset), std::abs(max_offset));
//     double buffer_distance =
//         max_absolute_offset * std::max(prev_scale, next_scale) +
//         METRIC_DISTANCE_0;
//     Segment_2 base_segment(main_start, main_end);

//     const std::vector<std::size_t> current_las_indices =
//         las_kd_tree.search_indices_in_box(base_segment.bbox(),
//                                           max_absolute_offset);

//     std::vector<PtsStructs::PointId> current_las_point_ids;
//     current_las_point_ids.reserve(current_las_indices.size());
//     for (std::size_t idx : current_las_indices) {
//         current_las_point_ids.emplace_back(PtsStructs::PointId(idx));
//     }

//     // Compute the weights for the LAS points
//     std::vector<double> weights;
//     compute_weights(las_points, current_las_point_ids, weights);

//     // Compute the metric for each offset
//     metrics.resize(offsets.size(), 0.0);
//     for (size_t i = 0; i < offsets.size(); ++i) {
//         double offset = offsets[i];
//         Point_2 new_main_start =
//             main_start + offset * prev_scale * prev_direction;
//         Point_2 new_main_end = main_end + offset * next_scale *
//         next_direction;

//         Roofprints::Edge new_main_edge(new_main_start, new_main_end);
//         if (new_main_edge.get_length() < 1e-6) {
//             std::cout << "The main edge is too small, skipping metric "
//                          "computation for this offset"
//                       << std::endl;
//             continue;
//         }
//         Roofprints::Edge new_prev_edge(prev_start, new_main_start);
//         Roofprints::Edge new_next_edge(new_main_end, next_end);

//         // Sum the scores for all points in the LAS point cloud
//         double total_score_main = 0.0;
//         double total_score_prev = 0.0;
//         double total_score_next = 0.0;
//         for (size_t j = 0; j < current_las_point_ids.size(); ++j) {
//             const PtsStructs::PointId las_point_id =
//             current_las_point_ids[j]; const Point_2 &point =
//             las_points->get_point_2d(las_point_id);

//             double score_main = compute_metric(point, new_main_edge);
//             double score_prev = compute_metric(point, new_prev_edge);
//             double score_next = compute_metric(point, new_next_edge);

//             total_score_main += score_main * weights[las_point_id];
//             total_score_prev += score_prev * weights[las_point_id];
//             total_score_next += score_next * weights[las_point_id];
//         }

//         double length_penalization_main =
//             std::min(new_main_edge.get_length(), MAX_LENGTH_PENALIZATION);
//         double length_penalization_prev =
//             std::min(new_prev_edge.get_length(), MAX_LENGTH_PENALIZATION);
//         double length_penalization_next =
//             std::min(new_next_edge.get_length(), MAX_LENGTH_PENALIZATION);

//         metrics[i] = (total_score_main / length_penalization_main) +
//                      (total_score_prev / length_penalization_prev) +
//                      (total_score_next / length_penalization_next);
//     }
// }

// //
// =============================================================================
// // Roofprints::EdgeMover
// //
// =============================================================================

// Roofprints::EdgeMover::EdgeMover(
//     const std::vector<EdgeSequenceWithNeighbours> &edges_with_neighbours,
//     const Vector_2 &moving_direction, AllPointsPtr all_points)
//     : edges_with_neighbours(edges_with_neighbours),
//       moving_direction(moving_direction), all_points(all_points) {}

// std::tuple<double, double>
// Roofprints::EdgeMover::_compute_movement_limits() const {
//     double min_offset = -std::numeric_limits<double>::infinity();
//     double max_offset = std::numeric_limits<double>::infinity();
//     for (const auto &edge : edges_with_neighbours) {
//         auto [edge_min_offset, edge_max_offset] =
//             edge.compute_movement_limits(moving_direction, all_points);
//         min_offset = std::max(min_offset, edge_min_offset);
//         max_offset = std::min(max_offset, edge_max_offset);
//     }
//     return {min_offset, max_offset};
// }

// double Roofprints::EdgeMover::compute_optimal_offset(
//     double max_absolute_offset, double offset_step, const KdTree_2
//     &las_kd_tree, PtsStructs::StoragePtr las_points) const { auto
//     [min_offset_limit, max_offset_limit] = _compute_movement_limits(); double
//     actual_min_offset = std::max(min_offset_limit, -max_absolute_offset);
//     double actual_max_offset = std::min(max_offset_limit,
//     max_absolute_offset);

//     // Build the list of offsets to evaluate
//     std::vector<double> offsets(0.0);
//     for (double offset = offset_step; offset <= actual_max_offset;
//          offset += offset_step) {
//         offsets.push_back(offset);
//     }
//     for (double offset = -offset_step; offset >= actual_min_offset;
//          offset -= offset_step) {
//         offsets.push_back(offset);
//     }

//     // Compute the metrics for all offsets
//     std::vector<double> metrics_sum;
//     for (const auto &edge_with_neighbours : edges_with_neighbours) {
//         std::vector<double> metrics;
//         edge_with_neighbours.compute_metric_for_offsets(
//             moving_direction, offsets, all_points, las_kd_tree, las_points,
//             metrics);
//         if (metrics_sum.empty()) {
//             metrics_sum = metrics;
//         } else {
//             for (size_t i = 0; i < metrics.size(); ++i) {
//                 metrics_sum[i] += metrics[i];
//             }
//         }
//     }

//     // Find the best offset
//     double best_offset = 0.0;
//     double best_metric = -std::numeric_limits<double>::infinity();
//     for (size_t i = 0; i < offsets.size(); ++i) {
//         if (metrics_sum[i] > best_metric) {
//             best_metric = metrics_sum[i];
//             best_offset = offsets[i];
//         }
//     }

//     return best_offset;
// }

// //
// =============================================================================
// // Roofprints::SuperOutline
// //
// =============================================================================

// Roofprints::SuperOutline::SuperOutline(
//     const std::vector<EdgeSequence> &edges,
//     // const std::vector<std::string> &edge_index_to_outline_id,
//     const std::vector<EdgeSequenceGroupId> &edge_index_to_edge_group_id,
//     const std::vector<std::vector<EdgeSequenceId>>
//         &edge_index_to_neighbour_edge_indices,
//     // AllPointsPtr all_points,
//     EdgeMatchingPtr all_lines)
//     : edges(edges),
//       // edge_index_to_outline_id(edge_index_to_outline_id),
//       edge_index_to_edge_group_id(edge_index_to_edge_group_id),
//       edge_index_to_neighbour_edge_indices(
//           edge_index_to_neighbour_edge_indices),
//       //   all_points(all_points),
//       all_lines(all_lines) {
//     // Build the backward mappings
//     auto num_edges = edges.size();
//     for (EdgeSequenceId i(0); i < num_edges; ++i) {
//         // // Outline ID
//         // const std::string &outline_id = edge_index_to_outline_id[i];
//         // outline_id_to_edge_indices[outline_id].push_back(i);

//         // Edge group ID
//         EdgeSequenceGroupId edge_group_id = edge_index_to_edge_group_id[i];
//         edge_group_id_to_edge_indices[edge_group_id].push_back(i);
//     }

//     // Compute lengths and the moving direction for each edge group
//     auto num_groups = edge_group_id_to_edge_indices.size();
//     edge_group_to_moving_direction.resize(num_groups);
//     std::vector<double> edge_lengths(num_groups);
//     for (EdgeSequenceGroupId i(0); i < num_groups; ++i) {
//         std::vector<EdgeSequenceId> &edge_indices =
//             edge_group_id_to_edge_indices[i];
//         double total_length = 0.0;
//         Vector_2 moving_direction(0.0, 0.0);
//         for (std::size_t j = 0; j < edge_indices.size(); ++j) {
//             // Get the current edge and its neighbours in the group
//             EdgeSequence edge_sequence = edges[edge_indices[j]];
//             EdgeSequence prev_edge_sequence =
//                 edges[edge_indices[(j + edge_indices.size() - 1) %
//                                    edge_indices.size()]];
//             EdgeSequence next_edge_sequence =
//                 edges[edge_indices[(j + 1) % edge_indices.size()]];

//             Point_2 start = all_points->get(edges[edge_index].get_start());
//             Point_2 end = all_points->get(edges[edge_index].get_end());

//             // Length
//             double length = CGAL::sqrt(CGAL::squared_distance(start, end));
//             total_length += length;

//             // Moving direction
//             Vector_2 edge_vec = end - start;
//             Vector_2 perp_vec;
//             if (edge_vec.x() < 0) {
//                 perp_vec = Vector_2(edge_vec.y(), -edge_vec.x());
//             } else {
//                 perp_vec = Vector_2(-edge_vec.y(), edge_vec.x());
//             }
//             perp_vec = perp_vec / std::sqrt(perp_vec.squared_length());
//             moving_direction += length * perp_vec;
//         }
//         moving_direction /= std::sqrt(moving_direction.squared_length());

//         // Check if the moving direction is normalized
//         if (std::abs(std::sqrt(moving_direction.squared_length()) - 1.0) >
//             1e-6) {
//             std::cerr << "First point of first edge: "
//                       << all_points->get(edges[edge_indices[0]].get_start())
//                       << std::endl;
//             std::cerr << "Moving direction: " << moving_direction <<
//             std::endl; std::cerr << "Moving direction length: "
//                       << std::sqrt(moving_direction.squared_length())
//                       << std::endl;
//             throw std::runtime_error("Moving direction is not normalized");
//         }

//         edge_lengths[i] = total_length;
//         edge_group_to_moving_direction[i] = moving_direction;
//     }

//     // Order the edge groups by initial length
//     edge_groups_ordered_by_initial_length.resize(
//         edge_group_id_to_edge_indices.size());
//     for (EdgeSequenceGroupId i(0); i < num_groups; ++i) {
//         edge_groups_ordered_by_initial_length[i] = i;
//     }
//     std::sort(edge_groups_ordered_by_initial_length.begin(),
//               edge_groups_ordered_by_initial_length.end(),
//               [&edge_lengths](EdgeSequenceGroupId a, EdgeSequenceGroupId b) {
//                   return edge_lengths[a] > edge_lengths[b];
//               });
// }

// void Roofprints::SuperOutline::optimize_edges(
//     const KdTree_2 &las_kd_tree, PtsStructs::StoragePtr las_points) const {

//     // Iterate over the edge sequences ordered by initial length
//     for (EdgeSequenceGroupId edge_group_index :
//          edge_groups_ordered_by_initial_length) {
//         // Get the edge group and its neighbours
//         const std::vector<EdgeSequenceId> &main_edge_indices =
//             edge_group_id_to_edge_indices.at(edge_group_index);
//         std::vector<std::tuple<EdgeSequenceId, EdgeSequenceId,
//         EdgeSequenceId>>
//             edges_with_neighbours_ids;
//         std::vector<EdgeSequenceWithNeighbours> edges_with_neighbours;
//         for (const EdgeSequenceId &main_edge_index : main_edge_indices) {
//             std::vector<EdgeSequenceId> neighbour_edge_indices =
//                 edge_index_to_neighbour_edge_indices[main_edge_index];
//             if (neighbour_edge_indices.size() != 2) {
//                 throw std::runtime_error("Each edge sequence should have "
//                                          "exactly 2 neighbour edge "
//                                          "sequences");
//             }
//             edges_with_neighbours_ids.push_back({neighbour_edge_indices[0],
//                                                  main_edge_index,
//                                                  neighbour_edge_indices[1]});
//             // std::cout << "Main edge index: " << main_edge_index
//             //           << ", neighbour edge indices: "
//             //           << neighbour_edge_indices[0] << ", "
//             //           << neighbour_edge_indices[1] << std::endl;
//             edges_with_neighbours.push_back(EdgeSequenceWithNeighbours(
//                 edges[main_edge_index], edges[neighbour_edge_indices[0]],
//                 edges[neighbour_edge_indices[1]]));
//         }

//         // Compute the optimal offset for the edge group
//         const Vector_2 &moving_direction =
//             edge_group_to_moving_direction[edge_group_index];
//         // std::cout << "Optimizing edge group " << edge_group_index
//         //           << " with moving direction: " << moving_direction
//         //           << std::endl;
//         const EdgeMover edge_mover(edges_with_neighbours, moving_direction,
//                                    all_points);

//         double optimal_offset = edge_mover.compute_optimal_offset(
//             MAX_ABSOLUTE_OFFSET, OFFSET_STEP, las_kd_tree, las_points);

//         std::map<EdgeSequenceId, std::vector<Point_2>>
//         edge_index_to_new_points; std::set<EdgeSequenceId>
//         updated_edge_indices;
//         // Update the points of the edge group and its neighbours
//         for (const auto &edge_with_neighbours_ids :
//         edges_with_neighbours_ids) {
//             const EdgeSequenceId &main_edge_id =
//                 std::get<1>(edge_with_neighbours_ids);
//             const EdgeSequenceId &previous_edge_id =
//                 std::get<0>(edge_with_neighbours_ids);
//             const EdgeSequenceId &next_edge_id =
//                 std::get<2>(edge_with_neighbours_ids);

//             for (const EdgeSequenceId &edge_id :
//                  {main_edge_id, previous_edge_id, next_edge_id}) {
//                 if (updated_edge_indices.count(edge_id) > 0) {
//                     throw std::runtime_error(
//                         "Edge sequence has already been updated");
//                 }
//             }

//             const EdgeSequence &main_edge = edges[main_edge_id];
//             const EdgeSequence &previous_edge = edges[previous_edge_id];
//             const EdgeSequence &next_edge = edges[next_edge_id];

//             // Get the points
//             const Point_2 &prev_start =
//                 all_points->get(previous_edge.get_start());
//             const Point_2 &main_start =
//             all_points->get(main_edge.get_start()); const Point_2 &main_end =
//             all_points->get(main_edge.get_end()); const Point_2 &next_end =
//             all_points->get(next_edge.get_end());

//             // Create the lines for the previous, main and next edges
//             const Line_2 prev_line(prev_start, main_start);
//             const Line_2 main_line(main_start, main_end);
//             const Line_2 next_line(main_end, next_end);

//             // Move the main line by the optimal offset in the moving
//             // direction
//             const Line_2 new_main_line(main_line.point(0) +
//                                            optimal_offset * moving_direction,
//                                        main_line.direction());

//             // Compute the new start and end points of the main edge
//             auto prev_intersection =
//                 CGAL::intersection(prev_line, new_main_line);
//             auto next_intersection =
//                 CGAL::intersection(next_line, new_main_line);
//             if (!prev_intersection || !next_intersection) {
//                 throw std::runtime_error("Failed to compute the intersection
//                 "
//                                          "of the new main line with "
//                                          "the previous or next line");
//             }
//             if (!std::holds_alternative<Point_2>(*prev_intersection) ||
//                 !std::holds_alternative<Point_2>(*next_intersection)) {
//                 throw std::runtime_error("Unexpected intersection type when "
//                                          "computing the new main line "
//                                          "intersections");
//             }
//             const Point_2 new_main_start =
//                 std::get<Point_2>(*prev_intersection);
//             const Point_2 new_main_end =
//             std::get<Point_2>(*next_intersection);

//             // Compute the new positions of the inner points of all three
//             // edges
//             std::vector<Point_2> new_previous_points;
//             previous_edge.compute_updated_points(
//                 prev_start, new_main_start, all_points, new_previous_points);
//             std::vector<Point_2> new_main_points;
//             main_edge.compute_updated_points(new_main_start, new_main_end,
//                                              all_points, new_main_points);
//             std::vector<Point_2> new_next_points;
//             next_edge.compute_updated_points(new_main_end, next_end,
//             all_points,
//                                              new_next_points);

//             // Update the points in the all_points structure
//             previous_edge.update_points(new_previous_points, all_points);
//             main_edge.update_points(new_main_points, all_points);
//             next_edge.update_points(new_next_points, all_points);

//             // Remember the updated edge indices
//             updated_edge_indices.insert(main_edge_id);
//             updated_edge_indices.insert(previous_edge_id);
//             updated_edge_indices.insert(next_edge_id);
//         }
//     }
// }

// //
// =============================================================================
// // Roofprint construction pipeline (free functions)
// //
// =============================================================================

// void select_outlines_in_las(
//     NewLasReader &reader,
//     const std::vector<MultiPolygonZWithAttributes> &outlines,
//     double buffer_distance,
//     std::vector<MultiPolygonZWithAttributes> &selected_outlines) {
//     selected_outlines.clear();

//     OGREnvelopePtr las_bounding_box = reader.points->bounding_box();
//     las_bounding_box->MinX += buffer_distance;
//     las_bounding_box->MinY += buffer_distance;
//     las_bounding_box->MaxX -= buffer_distance;
//     las_bounding_box->MaxY -= buffer_distance;

//     for (const auto &outline : outlines) {
//         OGREnvelopePtr outline_bounding_box = outline.bounding_box();

//         if (las_bounding_box->Contains(*outline_bounding_box.get())) {
//             selected_outlines.push_back(outline);
//         }
//     }
// }

// void group_collinear_edges_in_multipolygon(
//     const MultiPolygonZ &multi_polygon, std::vector<Point_2> &points,
//     std::vector<Roofprints::MovableLine> &lines,
//     std::vector<std::vector<Roofprints::EdgeSequence>>
//         &grouped_edge_sequences) {
//     // Output layout: one vector per polygon, each containing grouped edge
//     // sequences for that polygon.
//     grouped_edge_sequences.clear();

//     if (!multi_polygon.multi_polygon) {
//         return;
//     }

//     const int num_polygons = multi_polygon.multi_polygon->getNumGeometries();
//     grouped_edge_sequences.resize(num_polygons);

//     for (int polygon_idx = 0; polygon_idx < num_polygons; ++polygon_idx) {
//         // Access the polygon exterior ring used as the cyclic edge chain.
//         OGRPolygon *polygon = dynamic_cast<OGRPolygon *>(
//             multi_polygon.multi_polygon->getGeometryRef(polygon_idx));
//         if (!polygon) {
//             throw std::runtime_error(
//                 "Failed to cast geometry to polygon in multipolygon");
//         }

//         const OGRLinearRing *exterior_ring = polygon->getExteriorRing();
//         if (!exterior_ring) {
//             continue;
//         }

//         const int num_ring_points = exterior_ring->getNumPoints();
//         if (num_ring_points < 2) {
//             continue;
//         }

//         // OGR rings are usually closed (last point == first point). We keep
//         // a single copy of the first point for indexing the cyclic edge
//         // list.
//         std::size_t num_vertices = static_cast<std::size_t>(num_ring_points);
//         if (exterior_ring->getX(0) ==
//                 exterior_ring->getX(num_ring_points - 1) &&
//             exterior_ring->getY(0) ==
//                 exterior_ring->getY(num_ring_points - 1)) {
//             if (num_vertices <= 1) {
//                 continue;
//             }
//             --num_vertices;
//         }

//         if (num_vertices < 2) {
//             continue;
//         }

//         // Cache 2D ring vertices
//         std::vector<Point_2> ring_vertices;
//         ring_vertices.reserve(num_vertices);
//         for (std::size_t i = 0; i < num_vertices; ++i) {
//             ring_vertices.emplace_back(exterior_ring->getX(i),
//                                        exterior_ring->getY(i));
//         }

//         // Cache 2D edge vectors
//         std::vector<Roofprints::MovableLine> ring_lines;
//         ring_lines.reserve(num_vertices);
//         for (std::size_t i = 0; i < num_vertices; ++i) {
//             const std::size_t next_idx = (i + 1) % num_vertices;
//             Line_2 line(ring_vertices[i], ring_vertices[next_idx]);
//             ring_lines.emplace_back(ring_vertices[i], line);
//         }

//         // Initialize with the first ring edge, then greedily append or split
//         std::vector<std::vector<Roofprints::PointId>> point_ids{
//             {Roofprints::PointId(0), Roofprints::PointId(1)}};
//         std::vector<std::vector<Roofprints::LineId>> line_ids{
//             {Roofprints::LineId(0)}};

//         for (std::size_t edge_start_idx = 1; edge_start_idx < num_vertices;
//              ++edge_start_idx) {
//             const std::size_t current_end_idx =
//                 (edge_start_idx + 1) % num_vertices;

//             std::vector<Roofprints::PointId> last_point_ids =
//             point_ids.back();

//             Roofprints::PointId p0_idx =
//                 last_point_ids[last_point_ids.size() - 2];
//             Roofprints::PointId p1_idx =
//                 last_point_ids[last_point_ids.size() - 1];

//             const bool merge = should_merge_edge_sequences(
//                 ring_vertices[p0_idx], ring_vertices[p1_idx],
//                 ring_vertices[current_end_idx]);

//             if (merge) {
//                 // Keep all intermediate vertices inside the same sequence.
//                 point_ids.back().push_back(
//                     Roofprints::PointId(current_end_idx));
//                 line_ids.back().push_back(Roofprints::LineId(edge_start_idx));
//             } else {
//                 point_ids.emplace_back(std::vector<Roofprints::PointId>{
//                     Roofprints::PointId(edge_start_idx),
//                     Roofprints::PointId(current_end_idx)});
//                 line_ids.emplace_back(std::vector<Roofprints::LineId>{
//                     Roofprints::LineId(edge_start_idx)});
//             }
//         }

//         // Also test the cyclic boundary between the last and first
//         // sequences to preserve polygon continuity.
//         if (point_ids.size() > 1) {
//             const std::vector<Roofprints::PointId> &first_point_ids =
//                 point_ids.front();
//             const std::vector<Roofprints::PointId> &last_point_ids =
//                 point_ids.back();

//             const Roofprints::PointId p0_idx =
//                 last_point_ids[last_point_ids.size() - 2];
//             const Roofprints::PointId p1_idx =
//                 last_point_ids[last_point_ids.size() - 1];
//             const Roofprints::PointId p2_idx = first_point_ids[0];

//             const bool merge = should_merge_edge_sequences(
//                 ring_vertices[p0_idx], ring_vertices[p1_idx],
//                 ring_vertices[p2_idx]);

//             if (merge) {
//                 // Merge the first and last sequences by concatenating their
//                 // point ids
//                 std::vector<Roofprints::PointId> merged_point_ids =
//                     last_point_ids;
//                 merged_point_ids.insert(merged_point_ids.end(),
//                                         first_point_ids.begin() + 1,
//                                         first_point_ids.end());
//                 point_ids.front() = merged_point_ids;
//                 point_ids.pop_back();

//                 // Merge the line ids as well
//                 const std::vector<Roofprints::LineId> &first_line_ids =
//                     line_ids.front();
//                 const std::vector<Roofprints::LineId> &last_line_ids =
//                     line_ids.back();

//                 std::vector<Roofprints::LineId> merged_line_ids =
//                 last_line_ids; merged_line_ids.insert(merged_line_ids.end(),
//                                        first_line_ids.begin(),
//                                        first_line_ids.end());
//                 line_ids.front() = merged_line_ids;
//                 line_ids.pop_back();
//             }
//         }

//         std::vector<Roofprints::EdgeSequence> &edge_sequences =
//             grouped_edge_sequences[polygon_idx];

//         // Append the ring vertices to the global points list and update the
//         // indices in the edge sequences
//         const std::size_t global_point_offset = points.size();
//         points.insert(points.end(), ring_vertices.begin(),
//         ring_vertices.end()); for (Roofprints::EdgeSequence &edge_sequence :
//         edge_sequences) {
//             for (Roofprints::PointId &point_id : edge_sequence.point_ids) {
//                 point_id = Roofprints::PointId(point_id +
//                 global_point_offset);
//             }
//         }

//         // Append the ring lines to the global edges list and update the
//         indices
//         // in the edge sequences
//         const std::size_t global_line_offset = lines.size();
//         lines.insert(lines.end(), ring_lines.begin(), ring_lines.end());
//         for (Roofprints::EdgeSequence &edge_sequence : edge_sequences) {
//             for (Roofprints::LineId &line_id : edge_sequence.line_ids) {
//                 line_id = Roofprints::LineId(line_id + global_line_offset);
//             }
//         }
//     }
// }

// void _compute_roofprints(
//     NewLasReader &las_reader,
//     const std::vector<MultiPolygonZWithAttributes> &all_outlines,
//     double las_buffer_distance, double outline_buffer_distance,
//     std::vector<MultiPolygonZWithAttributes> &roofprints) {
//     roofprints.clear();

//     // Initialize the KD-tree for the LAS points
//     std::cout << "Initializing KD-tree for LAS points..." << std::endl;
//     KdTree_2 las_kd_tree = las_reader.points->build_kd_tree_2d();

//     // Select the outlines that are relevant for the LAS file
//     std::vector<MultiPolygonZWithAttributes> selected_outlines;
//     select_outlines_in_las(las_reader, all_outlines, las_buffer_distance,
//                            selected_outlines);
//     std::cout << "Selected " << selected_outlines.size()
//               << " outlines that are relevant for the LAS file." <<
//               std::endl;

//     if (selected_outlines.empty()) {
//         std::cout
//             << "No relevant outlines found, skipping roofprint computation."
//             << std::endl;
//         return;
//     }

//     // Extract and group the edges if they are almost collinear
//     std::cout << "Extracting and grouping edges from the selected
//     outlines..."
//               << std::endl;
//     std::size_t num_outlines = selected_outlines.size();
//     std::vector<Roofprints::EdgeSequence> edges;
//     std::vector<std::vector<std::vector<Roofprints::EdgeSequenceId>>>
//         outlines_multi_polygon_edge_sequences_ids(num_outlines);
//     std::vector<Point_2> points;
//     std::vector<Roofprints::MovableLine> lines;
//     for (std::size_t outline_idx = 0; outline_idx < selected_outlines.size();
//          ++outline_idx) {
//         const auto &outline = selected_outlines[outline_idx];
//         std::vector<std::vector<Roofprints::EdgeSequence>>
//             grouped_edge_sequences;
//         group_collinear_edges_in_multipolygon(outline, points, lines,
//                                               grouped_edge_sequences);

//         std::vector<std::vector<Roofprints::EdgeSequenceId>>
//         edge_sequences_ids(
//             grouped_edge_sequences.size());
//         for (std::size_t polygon_id = 0;
//              polygon_id < grouped_edge_sequences.size(); ++polygon_id) {
//             const auto &polygon_edge_sequences =
//                 grouped_edge_sequences[polygon_id];
//             for (const auto &edge_sequence : polygon_edge_sequences) {
//                 Roofprints::EdgeSequenceId edge_sequence_id(edges.size());
//                 edges.push_back(edge_sequence);
//                 edge_sequences_ids[polygon_id].push_back(edge_sequence_id);
//             }
//         }
//         outlines_multi_polygon_edge_sequences_ids[outline_idx] =
//             edge_sequences_ids;
//     }

//     // Initialize the AllPoints structure with the extracted points
//     std::cout << "Initializing points structure..." << std::endl;
//     Roofprints::AllPoints all_points(points);
//     Roofprints::AllPointsPtr all_points_ptr =
//         std::make_shared<Roofprints::AllPoints>(all_points);

//     // Initialize the EdgeMatching structure with the extracted lines
//     std::cout << "Initializing lines structure..." << std::endl;
//     Roofprints::EdgeMatching all_lines(lines);
//     Roofprints::EdgeMatchingPtr all_lines_ptr =
//         std::make_shared<Roofprints::EdgeMatching>(all_lines);

//     // Group the edge sequences with overlapping edges
//     std::cout << "Grouping edge sequences with overlapping edges..."
//               << std::endl;
//     // TODO

//     // Build the super outlines
//     std::cout << "Building super outlines..." << std::endl;
//     std::vector<Roofprints::SuperOutline> super_outlines;
//     for (std::size_t outline_idx = 0; outline_idx < num_outlines;
//          ++outline_idx) {
//         const auto &outline_edge_sequences_ids =
//             outlines_multi_polygon_edge_sequences_ids[outline_idx];
//         std::vector<Roofprints::EdgeSequence> edge_sequences;
//         std::vector<Roofprints::EdgeSequenceGroupId>
//             edge_index_to_edge_group_id;
//         std::vector<std::vector<Roofprints::EdgeSequenceId>>
//             edge_index_to_neighbour_edge_indices;

//         // Build the local neighbourhood information for each edge sequence
//         for (const auto &polygon_edge_sequences_ids :
//              outline_edge_sequences_ids) {
//             std::size_t num_edge_sequences =
//             polygon_edge_sequences_ids.size(); if (num_edge_sequences < 3) {
//                 std::cerr << "Polygon id: "
//                           << selected_outlines[outline_idx].get_id()
//                           << std::endl;
//                 throw std::runtime_error(
//                     "Each polygon should have at least 3 edge "
//                     "sequences");
//             }
//             Roofprints::EdgeSequenceId first_edge_sequence_id(
//                 edge_sequences.size());
//             for (std::size_t i = 0; i < num_edge_sequences; ++i) {
//                 const auto &edge_sequence =
//                     edges[polygon_edge_sequences_ids[i]];
//                 edge_sequences.push_back(edge_sequence);
//                 edge_index_to_edge_group_id.push_back(
//                     Roofprints::EdgeSequenceGroupId(edge_sequences.size() -
//                     1));

//                 Roofprints::EdgeSequenceId previous_edge_sequence_id(
//                     first_edge_sequence_id +
//                     (i + num_edge_sequences - 1) % num_edge_sequences);
//                 Roofprints::EdgeSequenceId next_edge_sequence_id(
//                     first_edge_sequence_id + (i + 1) % num_edge_sequences);
//                 edge_index_to_neighbour_edge_indices.push_back(
//                     {previous_edge_sequence_id, next_edge_sequence_id});
//             }
//         }

//         super_outlines.emplace_back(edge_sequences,
//         edge_index_to_edge_group_id,
//                                     edge_index_to_neighbour_edge_indices,
//                                     all_points_ptr);
//     }

//     // Optimize each super outline
//     std::cout << "Optimizing super outlines..." << std::endl;
//     ProgressBarTotal optimize_bar(
//         super_outlines.size(), "Optimizing super outlines",
//         indicators::option::ForegroundColor{indicators::Color::green});
//     optimize_bar.start();
//     for (auto &super_outline : super_outlines) {
//         super_outline.optimize_edges(las_kd_tree, las_reader.points);
//         optimize_bar.increment(1);
//     }
//     optimize_bar.finish();

//     // Rebuild the roofprints multi polygons
//     std::cout << "Rebuilding roofprints multi polygons..." << std::endl;
//     roofprints.clear();
//     roofprints.reserve(num_outlines);
//     for (std::size_t outline_idx = 0; outline_idx < selected_outlines.size();
//          ++outline_idx) {
//         const auto &outline = selected_outlines[outline_idx];
//         const auto &polygon_edge_sequences_ids =
//             outlines_multi_polygon_edge_sequences_ids[outline_idx];

//         // Build the polygons for the roofprint multi polygon
//         std::vector<PolygonZ> roofprint_polygons;
//         for (const auto &edge_sequences_ids : polygon_edge_sequences_ids) {
//             // Build a vector with the point ids of the polygons
//             std::vector<Roofprints::PointId> point_ids;
//             for (const auto &edge_sequence_id : edge_sequences_ids) {
//                 const auto &edge_sequence = edges[edge_sequence_id];
//                 // Avoid duplicate points at the boundaries of edge
//                 // sequences by skipping the last point of each edge
//                 // sequence
//                 point_ids.insert(point_ids.end(),
//                                  edge_sequence.point_ids.begin(),
//                                  edge_sequence.point_ids.end() - 1);
//             }

//             // Build the polygon
//             std::vector<Point_3> polygon_points;
//             for (const auto &point_id : point_ids) {
//                 Point_2 point_2d = all_points_ptr->get(point_id);
//                 polygon_points.emplace_back(point_2d.x(), point_2d.y(), 0.0);
//             }
//             roofprint_polygons.emplace_back(polygon_points, false);
//         }
//         roofprints.emplace_back(roofprint_polygons, outline.get_id(),
//                                 outline.get_outline_source());
//     }
// }

// void Roofprints::compute_roofprints(const std::string &input_las_file,
//                                     const std::string &input_bd_topo_file,
//                                     const std::string
//                                     &output_roofprints_file, double
//                                     las_buffer_distance, double
//                                     outline_buffer_distance, bool overwrite)
//                                     {

//     if (std::filesystem::exists(output_roofprints_file) && !overwrite) {
//         throw std::runtime_error("Output file already exists: " +
//                                  output_roofprints_file);
//     }

//     std::filesystem::create_directories(
//         std::filesystem::path(output_roofprints_file).parent_path());

//     // Read the LAS file and get the point view
//     std::cout << "Reading LAS file..." << std::endl;
//     NewLasReader las_reader(input_las_file);

//     // Read the building outlines from the BD TOPO file
//     std::vector<MultiPolygonZWithAttributes> all_outlines;
//     auto status =
//         read_building_outlines_from_bd_topo(input_bd_topo_file,
//         all_outlines);
//     if (!status.ok()) {
//         std::cerr << "Error reading BD TOPO: " << status.ToString()
//                   << std::endl;
//         throw std::runtime_error(
//             "Failed to read building outlines from BD TOPO");
//     }
//     std::cout << "Successfully read " << all_outlines.size()
//               << " building outlines from BD TOPO." << std::endl;

//     // Compute the roofprints
//     std::vector<MultiPolygonZWithAttributes> roofprints;
//     _compute_roofprints(las_reader, all_outlines, las_buffer_distance,
//                         outline_buffer_distance, roofprints);
//     std::cout << "Computed " << roofprints.size() << " roofprints."
//               << std::endl;

//     std::cout << "Transform Polygons into MultiPolygons" << std::endl;

//     // Write the roofprints to a Parquet file
//     std::cout << "Writing roofprints to Parquet file..." << std::endl;
//     auto write_status =
//         write_geoms_to_parquet(roofprints, output_roofprints_file,
//         overwrite);

//     if (!write_status.ok()) {
//         std::cerr << "Error writing roofprints to Parquet: "
//                   << write_status.ToString() << std::endl;
//         throw std::runtime_error("Failed to write roofprints to Parquet");
//     }
// }