// #pragma once

// #include <cstddef>
// #include <memory>
// #include <string>
// #include <vector>

// #include "geometry.hpp"
// #include "kd_tree.hpp"
// #include "las/reader.hpp"
// #include "points.hpp"
// #include "utils/cgal.hpp"
// #include "utils/strong_types.hpp"

// namespace Roofprints {

// // Strong type for point IDs to avoid confusion with other integer types
// struct PointIdTag {};
// typedef StrongType<PointIdTag, pdal::PointId> PointId;

// // Structure to store all the points once and refer to them by their IDs
// struct AllPoints {
//     std::vector<Point_2> points;

//     AllPoints(const std::vector<Point_2> &points);

//     Point_2 get(PointId point_id) const;
//     void set(PointId point_id, Point_2 point);
// };

// // We use shared pointers to the AllPoints structure to avoid copying it
// typedef std::shared_ptr<AllPoints> AllPointsPtr;

// struct Edge {
//   private:
//     Point_2 start;
//     Point_2 end;
//     Vector_2 direction;
//     double length;

//   public:
//     Edge(Point_2 start, Point_2 end)
//         : start(start), end(end), direction(end - start),
//           length(std::sqrt(direction.squared_length())) {
//         direction /= length;
//     }

//     Point_2 get_start() const { return start; }
//     Point_2 get_end() const { return end; }
//     Vector_2 get_direction() const { return direction; }
//     double get_length() const { return length; }
//     Segment_2 to_segment() const { return Segment_2(start, end); }
//     Line_2 to_line() const { return Line_2(start, end); }
// };

// struct MovableLine {
//   private:
//     Point_2 initial_start;
//     Line_2 line;
//     Vector_2 direction;

//   public:
//     MovableLine(Point_2 initial_start, Line_2 line);

//     MovableLine(MovableLine &&other) = default;
//     MovableLine(const MovableLine &other) = default;

//     Line_2 get_line() const;
//     Vector_2 get_direction() const;

//     void translate(Vector_2 offset);

//     Point_2 intersection(const Line_2 &other) const;
//     Point_2 intersection(const MovableLine &other) const;
// };

// // Strong type for line IDs to avoid confusion with other integer types
// struct LineIdTag {};
// typedef StrongType<LineIdTag, pdal::PointId> LineId;

// // Structure to store all the edges as lines and refer to them by their IDs
// struct AllLines {
//     std::vector<MovableLine> lines;

//     AllLines(const std::vector<MovableLine> &lines);

//     MovableLine get(LineId line_id) const;
//     void set(LineId line_id, MovableLine line);
// };

// // We use shared pointers to the AllLines structure to avoid copying it
// typedef std::shared_ptr<AllLines> AllLinesPtr;

// struct Polygon {
//     std::vector<PointId> point_ids;

//     Polygon(const std::vector<PointId> &point_ids) : point_ids(point_ids) {}
// };

// struct Outline {
//     Polygon outer;
//     std::vector<Polygon> holes;

//     Outline(const Polygon &outer, const std::vector<Polygon> &holes)
//         : outer(outer), holes(holes) {}
// };

// struct EdgeSequence {
//     // std::vector<PointId> point_ids;
//     std::vector<LineId> line_ids;

//     EdgeSequence() = default;
//     EdgeSequence(
//         // const std::vector<PointId> &point_ids,
//         const std::vector<LineId> &line_ids);

//     // PointId get_start() const;
//     // PointId get_end() const;

//     LineId get_first_line_id() const;
//     LineId get_last_line_id() const;
//     Point_2 get_first_point(Line_2 prev_line, AllLinesPtr all_lines) const;
//     Point_2 get_last_point(Line_2 next_line, AllLinesPtr all_lines) const;
//     Point_2 get_first_point(EdgeSequence prev_edge_sequence,
//                             AllLinesPtr all_lines) const;
//     Point_2 get_last_point(EdgeSequence next_edge_sequence,
//                            AllLinesPtr all_lines) const;

//     // void compute_updated_points(Point_2 new_start, Point_2 new_end,
//     //                             AllPointsPtr all_points,
//     //                             std::vector<Point_2> &new_points) const;
//     // void update_points(const std::vector<Point_2> &new_points,
//     //                    AllPointsPtr all_points) const;
//     void compute_updated_lines(Line_2 new_prev_line, Line_2 new_next_line,
//                                Vector_2 translation, AllLinesPtr all_lines,
//                                std::vector<MovableLine> &new_lines) const;
// };

// struct EdgeSequenceIdTag {};
// typedef StrongType<EdgeSequenceIdTag, std::size_t> EdgeSequenceId;

// // Structure to store an EdgeSequence along with its previous and next
// // EdgeSequences in the same polygon. We check that main.get_start() is equal
// to
// // previous.get_end() and main.get_end() is equal to next.get_start() when
// // creating an instance of this structure.
// struct EdgeSequenceWithNeighbours {
//     EdgeSequence main;
//     EdgeSequence prev;
//     EdgeSequence next;

//     EdgeSequenceWithNeighbours(const EdgeSequence &main,
//                                const EdgeSequence &neighbour_1,
//                                const EdgeSequence &neighbour_2);

//     /**
//      * @brief Compute the maximum and minimum offset for the moving
//      direction.
//      * The constraint for keeping the geometry valid is that we can move the
//      * main edges in a direction as long as none of the edges is reduced to a
//      * point.
//      *
//      * @param moving_direction The direction in which the edge sequence is
//      * moved. It should be a unit vector.
//      * @param all_points A pointer to the structure containing all points of
//      * the edge sequences.
//      * @return std::tuple<double, double> (min_offset, max_offset)
//      */
//     std::tuple<double, double>
//     compute_movement_limits(Vector_2 moving_direction,
//                             AllPointsPtr all_points) const;

//     void compute_metric_for_offsets(const Vector_2 &moving_direction,
//                                     const std::vector<double> &offsets,
//                                     AllPointsPtr points_outlines,
//                                     const KdTree_2 &las_kd_tree,
//                                     PtsStructs::StoragePtr las_points,
//                                     // const std::vector<double> &weights,
//                                     std::vector<double> &metrics) const;
// };

// struct EdgeMover {
//     AllPointsPtr all_points;
//     std::vector<EdgeSequenceWithNeighbours> edges_with_neighbours;
//     Vector_2 moving_direction;

//     EdgeMover(
//         const std::vector<EdgeSequenceWithNeighbours> &edges_with_neighbours,
//         const Vector_2 &moving_direction, AllPointsPtr all_points);

//     double compute_optimal_offset(double max_absolute_offset,
//                                   double offset_step,
//                                   const KdTree_2 &las_kd_tree,
//                                   PtsStructs::StoragePtr las_points) const;

//   private:
//     std::tuple<double, double> _compute_movement_limits() const;
// };

// struct EdgeSequenceGroupIdTag {};
// typedef StrongType<EdgeSequenceGroupIdTag, std::size_t> EdgeSequenceGroupId;

// // Structure to store the edges of the outlines and their relationships with
// the
// // outlines and with each other. Each SuperOutline should be the minimum set
// of
// // segments that must be optimized together to ensure the validity of the
// // geometry and the coherence of the touching edges.
// struct SuperOutline {
//     std::vector<EdgeSequence> edges;
//     // std::vector<std::string> edge_index_to_outline_id;
//     // std::map<std::string, std::vector<EdgeSequenceId>>
//     //     outline_id_to_edge_indices;
//     std::vector<EdgeSequenceGroupId> edge_index_to_edge_group_id;
//     std::map<EdgeSequenceGroupId, std::vector<EdgeSequenceId>>
//         edge_group_id_to_edge_indices;
//     std::vector<std::vector<EdgeSequenceId>>
//         edge_index_to_neighbour_edge_indices;
//     std::vector<EdgeSequenceGroupId> edge_groups_ordered_by_initial_length;
//     std::vector<Vector_2> edge_group_to_moving_direction;
//     // AllPointsPtr all_points;
//     AllLinesPtr all_lines;

//     SuperOutline(
//         const std::vector<EdgeSequence> &edges,
//         // const std::vector<std::string> &edge_index_to_outline_id,
//         const std::vector<EdgeSequenceGroupId> &edge_index_to_edge_group_id,
//         const std::vector<std::vector<EdgeSequenceId>>
//             &edge_index_to_neighbour_edge_indices,
//         // AllPointsPtr all_points,
//         AllLinesPtr all_lines);

//     void optimize_edges(const KdTree_2 &las_kd_tree,
//                         PtsStructs::StoragePtr las_points) const;
// };

// struct ComputeRoofprintsOptions {
//     std::string input_las_file;
//     std::string input_bd_topo_file;
//     std::string output_roofprints_file;
//     double las_buffer_distance;
//     double outline_buffer_distance;
//     bool overwrite;
// };

// void compute_roofprints(const std::string &input_las_file,
//                         const std::string &input_bd_topo_file,
//                         const std::string &output_roofprints_file,
//                         double las_buffer_distance,
//                         double outline_buffer_distance, bool overwrite);

// } // namespace Roofprints
