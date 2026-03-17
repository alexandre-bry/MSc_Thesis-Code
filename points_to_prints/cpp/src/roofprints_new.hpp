#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "geometry.hpp"
#include "points.hpp"
#include "utils/cgal.hpp"
#include "utils/strong_types.hpp"

namespace Roofprints {
struct PointIdTag {};
typedef StrongType<PointIdTag, pdal::PointId> PointId;

struct AllPoints {
    std::vector<Point_2> points;

    AllPoints(const std::vector<Point_2> &points);

    Point_2 get(PointId point_id) const;
    void set(PointId point_id, Point_2 point);
};

typedef std::shared_ptr<AllPoints> AllPointsPtr;

struct Polygon {
    std::vector<PointId> point_ids;

    Polygon(const std::vector<PointId> &point_ids) : point_ids(point_ids) {}
};

struct Outline {
    Polygon outer;
    std::vector<Polygon> holes;

    Outline(const Polygon &outer, const std::vector<Polygon> &holes)
        : outer(outer), holes(holes) {}
};

struct EdgeSequence {
    std::vector<PointId> point_ids;

    EdgeSequence(const std::vector<PointId> &point_ids);

    PointId get_start() const;
    PointId get_end() const;

    void compute_updated_points(Point_2 new_start, Point_2 new_end,
                                AllPointsPtr initial_points_ptr,
                                AllPointsPtr new_points_ptr,
                                std::vector<Point_2> &new_points);
};

struct EdgeSequenceIdTag {};
typedef StrongType<EdgeSequenceIdTag, std::size_t> EdgeSequenceId;

// Structure to store an EdgeSequence along with its previous and next
// EdgeSequences in the same polygon. We check that main.get_start() is equal to
// previous.get_end() and main.get_end() is equal to next.get_start() when
// creating an instance of this structure.
struct EdgeSequenceWithNeighbours {
    EdgeSequence main;
    EdgeSequence previous;
    EdgeSequence next;

    EdgeSequenceWithNeighbours(const EdgeSequence &main,
                               const EdgeSequence &previous,
                               const EdgeSequence &next);

    /**
     * @brief Compute the maximum and minimum offset for the moving direction.
     * The constraint for keeping the geometry valid is that we can move the
     * main edges in a direction as long as none of the edges is reduced to a
     * point.
     *
     * @param moving_direction The direction in which the edge sequence is
     * moved. It should be a unit vector.
     * @param all_points_ptr A pointer to the structure containing all points of
     * the edge sequences.
     * @return std::tuple<double, double> (min_offset, max_offset)
     */
    std::tuple<double, double>
    compute_movement_limits(Vector_2 moving_direction,
                            AllPointsPtr all_points_ptr) const;

    void compute_metric_for_offsets(
        const Vector_2 &moving_direction, const std::vector<double> &offsets,
        AllPointsPtr points_outlines,
        const std::vector<PtsStructs::PointId> &point_cloud_point_ids,
        const std::vector<double> &point_cloud_point_weights,
        PtsStructs::StoragePtr points_point_cloud,
        std::vector<double> &metrics) const;
};

struct EdgeMover {
    AllPointsPtr all_points_ptr;
    std::vector<EdgeSequenceWithNeighbours> edges_with_neighbours;
    Vector_2 moving_direction;

    EdgeMover(
        const std::vector<EdgeSequenceWithNeighbours> &edges_with_neighbours,
        const Vector_2 &moving_direction, AllPointsPtr all_points_ptr);

    void optimize(double max_absolute_offset, double offset_step,
                  const std::vector<PtsStructs::PointId> &point_cloud_point_ids,
                  const std::vector<double> &point_cloud_point_weights,
                  PtsStructs::StoragePtr points_point_cloud) const;

  private:
    std::tuple<double, double> _compute_movement_limits() const;
    void _compute_metric_for_offset(double offset) const;
};

// Structure to store the edges of the outlines and their relationships with the
// outlines and with each other. Each SuperOutline should be the minimum set of
// segments that must be optimized together to ensure the validity of the
// geometry and the coherence of the touching edges.
struct SuperOutline {
    std::vector<EdgeSequence> edges;
    std::vector<std::string> edge_index_to_outline_id;
    std::map<std::string, std::vector<EdgeSequenceId>>
        outline_id_to_edge_indices;
    std::vector<unsigned long> edge_index_to_edge_group_id;
    std::map<unsigned long, std::vector<EdgeSequenceId>>
        edge_group_id_to_edge_indices;
    std::vector<std::vector<EdgeSequenceId>>
        edge_index_to_neighbour_edge_indices;
    std::vector<EdgeSequenceId> edge_indices_ordered_by_initial_length;

    SuperOutline(const std::vector<EdgeSequence> &edges,
                 const std::vector<std::string> &edge_index_to_outline_id,
                 const std::vector<unsigned long> &edge_index_to_edge_group_id,
                 const std::vector<std::vector<EdgeSequenceId>>
                     &edge_index_to_neighbour_edge_indices);

    void optimize_edges(AllPointsPtr all_points_ptr);
};

} // namespace Roofprints
