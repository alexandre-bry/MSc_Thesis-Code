#include "transfer_3d.hpp"

#include <cstddef>
#include <iostream>
#include <ostream>
#include <random>

#include "../las/reader.hpp"
#include "../parquet.hpp"
#include "../parquet/reader.hpp"
#include "../points.hpp"
#include "../utils/pbar.hpp"

const double DISTANCE_SEGMENTS = 1.0;
const double MIN_COVERAGE_SEGMENT = 0.1;

bool SimpleRANSAC3D::_is_different_enough(
    const RansacSegment3D &s1,
    const std::vector<RansacSegment3D> &selected_segments) const {
    std::vector<std::pair<double, double>> covered_segments(
        {{s1.min_projection_value, s1.max_projection_value}});
    for (const RansacSegment3D &s2 : selected_segments) {
        double s2_min = s2.min_projection_value;
        double s2_max = s2.max_projection_value;
        std::vector<std::pair<double, double>> new_covered_segments;
        for (const auto &[cov_min, cov_max] : covered_segments) {
            if (s2_max < cov_min || s2_min > cov_max) {
                // No overlap
                new_covered_segments.emplace_back(cov_min, cov_max);
            } else {
                // Overlap exists, we need to split the covered segment
                if (s2_min > cov_min) {
                    new_covered_segments.emplace_back(cov_min, s2_min);
                }
                if (s2_max < cov_max) {
                    new_covered_segments.emplace_back(s2_max, cov_max);
                }
            }
        }
        covered_segments = new_covered_segments;
    }

    double total_covered_length = 0.0;
    for (const auto &[cov_min, cov_max] : covered_segments) {
        total_covered_length += cov_max - cov_min;
    }
    return total_covered_length > MIN_COVERAGE_SEGMENT;
}

double SimpleRANSAC3D::_similarity_inliers(const RansacSegment3D &s1,
                                           const RansacSegment3D &s2) const {
    std::unordered_set<std::size_t> inliers_s1(s1.inliers_indices.begin(),
                                               s1.inliers_indices.end());
    int shared_inliers = 0;
    for (std::size_t idx : s2.inliers_indices) {
        if (inliers_s1.count(idx) > 0) {
            shared_inliers++;
        }
    }
    int min_inliers =
        std::min(s1.inliers_indices.size(), s2.inliers_indices.size());
    return min_inliers > 0 ? static_cast<double>(shared_inliers) / min_inliers
                           : 0.0;
}

double SimpleRANSAC3D::_optimize_segment(const RansacSegment3D &segment) const {
    return 0.0;
}

SimpleRANSAC3D::SimpleRANSAC3D(const std::vector<Point_3> &_points,
                               const Segment_2 &_base_segment, int _samples,
                               double _distance_threshold)
    : base_segment(_base_segment), samples(_samples),
      distance_threshold(_distance_threshold) {
    gen = std::mt19937(rd());

    const Point_2 &seg_start = _base_segment.source();
    const Point_2 &seg_end = _base_segment.target();

    // Compute the projection of each point on the base segment to know
    // where it lies in 1D along the segment
    Line_2 base_line(seg_start, seg_end);
    UnitVector_2 base_direction(base_line.to_vector());

    std::vector<double> points_projection_value_unsorted(_points.size());
    for (std::size_t i = 0; i < _points.size(); ++i) {
        const Point_3 &point = _points[i];
        const Point_2 point_2d(point.x(), point.y());
        const Point_2 projection = base_line.projection(point_2d);
        points_projection_value_unsorted[i] =
            (_base_segment.target() - projection) * base_direction;
    }

    // Sort the points by their projection value
    std::vector<std::size_t> points_indices(_points.size());
    std::iota(points_indices.begin(), points_indices.end(), 0);
    std::sort(
        points_indices.begin(), points_indices.end(),
        [points_projection_value_unsorted](std::size_t idx1, std::size_t idx2) {
            return points_projection_value_unsorted[idx1] <
                   points_projection_value_unsorted[idx2];
        });
    points.resize(_points.size());
    points_projection_value.resize(_points.size());
    for (std::size_t i = 0; i < _points.size(); ++i) {
        points[i] = _points[points_indices[i]];
        points_projection_value[i] =
            points_projection_value_unsorted[points_indices[i]];
    }
}

void SimpleRANSAC3D::process_line(const Line_3 &line) {
    // Compute the score of each point for this line and keep track of the
    // inliers (points with non-zero score)
    std::vector<std::size_t> inliers_indices;
    std::vector<double> inliers_scores;
    double total_score = 0.0;
    double distance_threshold_sq = distance_threshold * distance_threshold;
    for (std::size_t point_idx = 0; point_idx < points.size(); ++point_idx) {
        const Point_3 &point = points[point_idx];
        double distance_sq = CGAL::squared_distance(point, line);
        double point_score = 0.0;
        if (distance_sq < distance_threshold_sq) {
            double distance = std::sqrt(distance_sq);
            point_score = 1 - (distance / distance_threshold);
            // point_score *= point_score;
            inliers_indices.push_back(point_idx);
            inliers_scores.push_back(point_score);
            total_score += point_score;
        }
    }

    if (inliers_indices.empty() || total_score <= 5.0) {
        return;
    }

    // Identify the different segments of inliers
    std::vector<std::vector<std::size_t>> all_segments_indices;
    std::vector<std::vector<double>> all_segments_scores;
    std::vector<std::pair<double, double>> all_segments_bounding_boxes;
    std::vector<std::size_t> current_segment_indices({inliers_indices[0]});
    std::vector<double> current_segment_scores({inliers_scores[0]});
    double current_projection_value =
        points_projection_value[inliers_indices[0]];
    for (std::size_t i = 1; i < inliers_indices.size(); ++i) {
        std::size_t point_idx = inliers_indices[i];
        double point_score = inliers_scores[i];

        // Check if the next point is far enough in terms of projection
        // value to start a new segment
        double projection_value = points_projection_value[point_idx];
        if (projection_value - current_projection_value > DISTANCE_SEGMENTS) {
            all_segments_indices.emplace_back(current_segment_indices);
            all_segments_scores.push_back(current_segment_scores);
            double min_proj =
                points_projection_value[current_segment_indices[0]];
            double max_proj =
                points_projection_value[current_segment_indices.back()];
            all_segments_bounding_boxes.emplace_back(min_proj, max_proj);

            // Start a new segment
            current_segment_indices.clear();
            current_segment_scores.clear();
        }
        // Add the point to the current segment and update
        // the score of the segment
        current_segment_indices.push_back(point_idx);
        current_segment_scores.push_back(point_score);

        current_projection_value = projection_value;
    }
    if (!current_segment_indices.empty()) {
        all_segments_indices.emplace_back(current_segment_indices);
        all_segments_scores.push_back(current_segment_scores);
        double min_proj = points_projection_value[current_segment_indices[0]];
        double max_proj =
            points_projection_value[current_segment_indices.back()];
        all_segments_bounding_boxes.emplace_back(min_proj, max_proj);
    }

    // Add the new segments to the current segments and sort them by score
    for (std::size_t i = 0; i < all_segments_indices.size(); ++i) {
        auto [min_proj, max_proj] = all_segments_bounding_boxes[i];
        best_segments.emplace_back(line, all_segments_indices[i],
                                   all_segments_scores[i], min_proj, max_proj);
    }

    // Sort the segments by score
    sort(best_segments.begin(), best_segments.end(),
         [this](const RansacSegment3D &s1, const RansacSegment3D &s2) {
             return s1.total_score > s2.total_score;
         });

    // Update the best segments
    std::vector<RansacSegment3D> selected_segments;
    for (const RansacSegment3D &segment : best_segments) {
        if (segment.total_score <= 5.0) {
            break;
        }
        // Check if the segment is similar to one of the already selected
        // segments and if so, we skip it
        bool too_similar = false;
        for (const RansacSegment3D &selected_segment : selected_segments) {
            if (this->_similarity_inliers(segment, selected_segment) > 0.5) {
                too_similar = true;
                break;
            }
        }
        if (too_similar) {
            continue;
        }
        if (_is_different_enough(segment, selected_segments)) {
            selected_segments.push_back(segment);
        }
    }
    best_segments = selected_segments;
}

void SimpleRANSAC3D::run() {
    std::uniform_int_distribution<std::size_t> dist(0, points.size() - 1);
    for (int i = 0; i < samples; ++i) {
        // Randomly sample two distinct points
        std::size_t idx1 = dist(gen);
        std::size_t idx2;
        do {
            idx2 = dist(gen);
        } while (idx2 == idx1);

        // Create a line from the two points and process it
        Line_3 line(points[idx1], points[idx2]);
        process_line(line);
    }
}

namespace Intersections1D {
enum class IntersectionType { NoIntersection, PartialOverlap, FullOverlap };
enum class FirstSegmentRelation { Before, After, Covers, IsCovered, None };

inline std::tuple<IntersectionType, FirstSegmentRelation>
intersection(std::pair<double, double> segment1,
             std::pair<double, double> segment2, double epsilon = 1e-9) {
    double s1_start = segment1.first - epsilon;
    double s1_end = segment1.second + epsilon;
    double s2_start = segment2.first - epsilon;
    double s2_end = segment2.second + epsilon;

    if (s1_end <= s2_start) {
        return {IntersectionType::NoIntersection, FirstSegmentRelation::Before};
    }
    if (s1_start >= s2_end) {
        return {IntersectionType::NoIntersection, FirstSegmentRelation::After};
    }
    if (s1_start >= s2_start) {
        if (s1_end <= s2_end) {
            return {IntersectionType::FullOverlap,
                    FirstSegmentRelation::IsCovered};
        } else {
            return {IntersectionType::PartialOverlap,
                    FirstSegmentRelation::After};
        }
    } else {
        if (s1_end <= s2_end) {
            return {IntersectionType::PartialOverlap,
                    FirstSegmentRelation::Before};
        } else {
            return {IntersectionType::FullOverlap,
                    FirstSegmentRelation::Covers};
        }
    }
}
} // namespace Intersections1D

struct Segment2DSpace {
  private:
    UnitVector_3 horizontal_direction;
    UnitVector_3 vertical_direction;
    Point_3 origin;
    double min_proj_x;
    double max_proj_x;
    std::vector<std::tuple<double, std::optional<std::size_t>, double,
                           std::optional<std::size_t>>>
        empty_spaces;
    std::vector<std::pair<Point_2, Point_2>> current_segments;

    std::pair<bool, bool>
    _crop_segment(const std::pair<Point_2, Point_2> &segment, double min_x,
                  double max_x,
                  std::pair<Point_2, Point_2> &cropped_segment) const {
        if (min_x > max_x) {
            std::string error_message =
                "min_x should be less than or equal to max_x: min_x = " +
                std::to_string(min_x) + ", max_x = " + std::to_string(max_x);
            throw std::invalid_argument(error_message);
        }

        cropped_segment = segment;

        Point_2 seg_start = cropped_segment.first;
        Point_2 seg_end = cropped_segment.second;
        if (seg_start.x() > seg_end.x()) {
            std::swap(seg_start, seg_end);
        }

        bool crop_left = seg_start.x() < min_x;
        bool crop_right = seg_end.x() > max_x;

        if (crop_left) {
            double new_start_x = min_x;
            double new_start_y =
                seg_start.y() + (seg_end.y() - seg_start.y()) *
                                    (new_start_x - seg_start.x()) /
                                    (seg_end.x() - seg_start.x());
            cropped_segment.first = Point_2(new_start_x, new_start_y);
        }
        if (crop_right) {
            double new_end_x = max_x;
            double new_end_y =
                seg_start.y() + (seg_end.y() - seg_start.y()) *
                                    (new_end_x - seg_start.x()) /
                                    (seg_end.x() - seg_start.x());
            cropped_segment.second = Point_2(new_end_x, new_end_y);
        }

        return {crop_left, crop_right};
    }

    std::pair<bool, bool>
    _extend_segment(const std::pair<Point_2, Point_2> &segment, double min_x,
                    double max_x,
                    std::pair<Point_2, Point_2> &extended_segment) const {
        if (min_x > max_x) {
            std::string error_message =
                "min_x should be less than or equal to max_x: min_x = " +
                std::to_string(min_x) + ", max_x = " + std::to_string(max_x);
            throw std::invalid_argument(error_message);
        }

        extended_segment = segment;

        Point_2 seg_start = extended_segment.first;
        Point_2 seg_end = extended_segment.second;
        if (seg_start.x() > seg_end.x()) {
            std::swap(seg_start, seg_end);
        }

        bool extend_left = seg_start.x() > min_x;
        bool extend_right = seg_end.x() < max_x;

        if (extend_left) {
            double new_start_x = min_x;
            double new_start_y =
                seg_start.y() + (seg_end.y() - seg_start.y()) *
                                    (new_start_x - seg_start.x()) /
                                    (seg_end.x() - seg_start.x());
            extended_segment.first = Point_2(new_start_x, new_start_y);
        }
        if (extend_right) {
            double new_end_x = max_x;
            double new_end_y =
                seg_start.y() + (seg_end.y() - seg_start.y()) *
                                    (new_end_x - seg_start.x()) /
                                    (seg_end.x() - seg_start.x());
            extended_segment.second = Point_2(new_end_x, new_end_y);
        }

        return {extend_left, extend_right};
    }

    /**
     * @brief Handle the intersection between two segments that are expected to
     * overlap but the strong segment should not cover the weak segment.
     * The idea is that if the two segments intersect in 2D, we use the
     * intersection point to split them, otherwise the strong one takes
     * precedence in the area they share on the X axis.
     *
     * @param strong_segment The segment with the priority over the other one
     * @param weak_segment The segment with lower priority
     * @param strong_segment_updated The updated strong segment after handling
     * the intersection
     * @param weak_segment_updated The updated weak segment after handling the
     * intersection
     * @param before Whether the strong segment is expected to be before the
     * weak segment on the X axis
     */
    void
    _handle_intersection(const std::pair<Point_2, Point_2> &strong_segment,
                         const std::pair<Point_2, Point_2> &weak_segment,
                         std::pair<Point_2, Point_2> &strong_segment_updated,
                         std::pair<Point_2, Point_2> &weak_segment_updated,
                         bool before) const {

        // Checks to see if we're in the expected situation of partial overlap
        // on the X axis
        auto [intersection_type, strong_segment_relation] =
            Intersections1D::intersection(
                {strong_segment.first.x(), strong_segment.second.x()},
                {weak_segment.first.x(), weak_segment.second.x()});

        if (intersection_type ==
            Intersections1D::IntersectionType::NoIntersection) {
            std::string error_message =
                "Segments are expected to overlap on the X axis but they "
                "don't. "
                "Strong segment: [" +
                std::to_string(strong_segment.first.x()) + ", " +
                std::to_string(strong_segment.second.x()) +
                "], "
                "Weak segment: [" +
                std::to_string(weak_segment.first.x()) + ", " +
                std::to_string(weak_segment.second.x()) + "]";
            throw std::logic_error(error_message);
        }
        if (intersection_type ==
                Intersections1D::IntersectionType::FullOverlap &&
            strong_segment_relation ==
                Intersections1D::FirstSegmentRelation::Covers) {
            std::string error_message =
                "Segments are expected to overlap on the X axis but the strong "
                "segment should not fully cover the weak segment. "
                "Strong segment: [" +
                std::to_string(strong_segment.first.x()) + ", " +
                std::to_string(strong_segment.second.x()) +
                "], "
                "Weak segment: [" +
                std::to_string(weak_segment.first.x()) + ", " +
                std::to_string(weak_segment.second.x()) + "]";
            throw std::logic_error(error_message);
        }
        if (intersection_type ==
            Intersections1D::IntersectionType::PartialOverlap) {
            if (before && strong_segment_relation ==
                              Intersections1D::FirstSegmentRelation::After) {
                std::string error_message =
                    "Segments are expected to overlap on the X axis but the "
                    "strong "
                    "segment should be before the weak segment. "
                    "Strong segment: [" +
                    std::to_string(strong_segment.first.x()) + ", " +
                    std::to_string(strong_segment.second.x()) +
                    "], "
                    "Weak segment: [" +
                    std::to_string(weak_segment.first.x()) + ", " +
                    std::to_string(weak_segment.second.x()) + "]";
                throw std::logic_error(error_message);
            }
            if (!before && strong_segment_relation ==
                               Intersections1D::FirstSegmentRelation::Before) {
                std::string error_message =
                    "Segments are expected to overlap on the X axis but the "
                    "strong "
                    "segment should be after the weak segment. "
                    "Strong segment: [" +
                    std::to_string(strong_segment.first.x()) + ", " +
                    std::to_string(strong_segment.second.x()) +
                    "], "
                    "Weak segment: [" +
                    std::to_string(weak_segment.first.x()) + ", " +
                    std::to_string(weak_segment.second.x()) + "]";
                throw std::logic_error(error_message);
            }
        }

        // Compute the intersection between the two segments in 2D
        auto result = CGAL::intersection(
            Segment_2(strong_segment.first, strong_segment.second),
            Segment_2(weak_segment.first, weak_segment.second));
        if (result) {
            if (const Point_2 *intersection_point =
                    std::get_if<Point_2>(&*result)) {
                // The segments intersect in 2D, we split them both at the
                // intersection point

                if (before) {
                    // The left part of the strong segment is the one that is
                    // kept
                    strong_segment_updated.first = strong_segment.first;
                    strong_segment_updated.second = *intersection_point;
                    weak_segment_updated.first = *intersection_point;
                    weak_segment_updated.second = weak_segment.second;
                } else {
                    // The right part of the strong segment is the one that is
                    // kept
                    weak_segment_updated.first = weak_segment.first;
                    weak_segment_updated.second = *intersection_point;
                    strong_segment_updated.first = *intersection_point;
                    strong_segment_updated.second = strong_segment.second;
                }

                return;
            }
        }

        // The segments are collinear and overlapping in 2D or they don't
        // intersect: we treat both cases the same by giving the priority to the
        // strong segment in the area they share on the X axis.
        std::pair<Point_2, Point_2> weak_segment_cropped;
        if (before) {
            // The strong segment is before the weak segment, so we crop
            // the left part of the weak segment
            auto [crop_left, crop_right] = _crop_segment(
                weak_segment, strong_segment.second.x(),
                std::numeric_limits<double>::infinity(), weak_segment_cropped);
        } else {
            // The strong segment is after the weak segment, so we crop
            // the right part of the weak segment
            auto [crop_left, crop_right] = _crop_segment(
                weak_segment, -std::numeric_limits<double>::infinity(),
                strong_segment.first.x(), weak_segment_cropped);
        }

        weak_segment_updated = weak_segment_cropped;
        strong_segment_updated = strong_segment;
    }

    double _get_projection_x(const Point_3 &point) const {
        return (point - origin) * horizontal_direction;
    }

    double _get_projection_y(const Point_3 &point) const {
        return (point - origin) * vertical_direction;
    }

    Point_2 _get_projection(const Point_3 &point) const {
        return Point_2(_get_projection_x(point), _get_projection_y(point));
    }

  public:
    Segment2DSpace(const Segment_2 &segment) {
        Point_2 seg_start = segment.source();
        Point_2 seg_end = segment.target();
        UnitVector_3 horizontal_dir(seg_end.x() - seg_start.x(),
                                    seg_end.y() - seg_start.y(), 0);
        UnitVector_3 vertical_dir(0, 0, 1);
        horizontal_direction = horizontal_dir;
        vertical_direction = vertical_dir;

        origin = Point_3(seg_start.x(), seg_start.y(), 0);
        min_proj_x = 0;
        max_proj_x = _get_projection_x(Point_3(seg_end.x(), seg_end.y(), 0));
        empty_spaces = {{min_proj_x, std::nullopt, max_proj_x, std::nullopt}};
    }

    /**
     * @brief Handles the addition of a new segment in the space, by merging
     * it with the existing segments it overlaps with and updating the empty
     * spaces accordingly.
     *
     * @param segment The new segment to add, in 3D.
     */
    void add_segment(const Segment_3 &segment) {

        // Project the segment on the 2D space
        Point_3 seg_start = segment.source();
        Point_3 seg_end = segment.target();
        Point_2 proj_start = _get_projection(seg_start);
        Point_2 proj_end = _get_projection(seg_end);
        if (proj_start.x() > proj_end.x()) {
            std::swap(proj_start, proj_end);
        }

        // Find all the empty spaces it overlaps with
        std::vector<bool> overlapped_empty_spaces(empty_spaces.size(), false);
        for (std::size_t i = 0; i < empty_spaces.size(); ++i) {
            const auto &[empty_min, min_seg_idx, empty_max, max_seg_idx] =
                empty_spaces[i];
            if (proj_end.x() < empty_min || proj_start.x() > empty_max) {
                continue;
            }
            overlapped_empty_spaces[i] = true;
        }

        std::vector<std::tuple<double, std::optional<std::size_t>, double,
                               std::optional<std::size_t>>>
            new_empty_spaces;

        for (std::size_t i = 0; i < empty_spaces.size(); ++i) {
            auto [empty_min, min_seg_idx, empty_max, max_seg_idx] =
                empty_spaces[i];
            if (!overlapped_empty_spaces[i]) {
                new_empty_spaces.emplace_back(empty_min, min_seg_idx, empty_max,
                                              max_seg_idx);
            } else {
                // The new segment overlaps with this empty space
                // We need to find if it reaches the left and/or right
                // border of the empty space and if there was a
                // previous/next segment
                // std::cout << "New segment with coordinates (" <<
                // proj_start.x()
                //           << ", " << proj_start.y() << ") to (" <<
                //           proj_end.x()
                //           << ", " << proj_end.y()
                //           << ") overlaps with empty space [" << empty_min
                //           << ", " << empty_max << "] with neighbours ("
                //           << (min_seg_idx.has_value()
                //                   ? std::to_string(min_seg_idx.value())
                //                   : "none")
                //           << ", "
                //           << (max_seg_idx.has_value()
                //                   ? std::to_string(max_seg_idx.value())
                //                   : "none")
                //           << ")" << std::endl;

                bool reaches_left_border = proj_start.x() <= empty_min;
                bool reaches_right_border = proj_end.x() >= empty_max;
                bool has_neighbour_left =
                    reaches_left_border && min_seg_idx.has_value();
                bool has_neighbour_right =
                    reaches_right_border && max_seg_idx.has_value();

                std::pair<Point_2, Point_2> new_segment(proj_start, proj_end);

                if (has_neighbour_left) {
                    // The new segment overlaps with the previous segment
                    // but not with the next one, so we can merge it with
                    // the previous segment

                    // Handle the intersection between the new segment and
                    // the previous segment

                    // std::cout << "Merging with left neighbour segment"
                    //           << std::endl;
                    // std::cout << "empty_min: " << empty_min << std::endl;

                    std::pair<Point_2, Point_2> existing_segment =
                        current_segments[min_seg_idx.value()];
                    std::pair<Point_2, Point_2> final_existing_segment;
                    std::pair<Point_2, Point_2> final_new_segment;
                    _handle_intersection(existing_segment, new_segment,
                                         final_existing_segment,
                                         final_new_segment, true);

                    // Update the existing segment
                    current_segments[min_seg_idx.value()] =
                        final_existing_segment;
                    empty_min = final_existing_segment.second.x();

                    // std::cout << "Updated existing segment to: ("
                    //           << final_existing_segment.first << ", "
                    //           << final_existing_segment.second << ")"
                    //           << std::endl;
                    // std::cout << "Updated empty_min to: " << empty_min
                    //           << std::endl;
                    // std::cout << "Changed new segment to: ("
                    //           << final_new_segment.first << ", "
                    //           << final_new_segment.second << ")" <<
                    //           std::endl;

                    new_segment = final_new_segment;
                }
                if (has_neighbour_right) {
                    // The new segment overlaps with the next segment but
                    // not with the previous one, so we can merge it with
                    // the next segment

                    // Handle the intersection between the new segment and
                    // the next segment

                    // std::cout << "Merging with right neighbour segment"
                    //           << std::endl;
                    // std::cout << "empty_max: " << empty_max << std::endl;

                    std::pair<Point_2, Point_2> existing_segment =
                        current_segments[max_seg_idx.value()];
                    std::pair<Point_2, Point_2> final_existing_segment;
                    std::pair<Point_2, Point_2> final_new_segment;
                    _handle_intersection(existing_segment, new_segment,
                                         final_existing_segment,
                                         final_new_segment, false);

                    // Update the existing segment
                    current_segments[max_seg_idx.value()] =
                        final_existing_segment;
                    empty_max = final_existing_segment.first.x();

                    // std::cout << "Updated existing segment to: ("
                    //           << final_existing_segment.first << ", "
                    //           << final_existing_segment.second << ")"
                    //           << std::endl;
                    // std::cout << "Updated empty_max to: " << empty_max
                    //           << std::endl;
                    // std::cout << "Changed new segment to: ("
                    //           << final_new_segment.first << ", "
                    //           << final_new_segment.second << ")" <<
                    //           std::endl;

                    new_segment = final_new_segment;
                }
                // Crop the new segment to the empty space if it exceeds it
                auto [crop_left, crop_right] = _crop_segment(
                    new_segment, empty_min, empty_max, new_segment);

                // Store the new segment
                auto new_segment_index = current_segments.size();
                current_segments.push_back(new_segment);

                // Add the remaining empty spaces (cropping means that the
                // new segment covered the empty space)
                if (!crop_left && !has_neighbour_left) {
                    double left_empty_min = empty_min;
                    double left_empty_max = new_segment.first.x();
                    new_empty_spaces.push_back({left_empty_min, min_seg_idx,
                                                left_empty_max,
                                                new_segment_index});
                }
                if (!crop_right && !has_neighbour_right) {
                    double right_empty_min = new_segment.second.x();
                    double right_empty_max = empty_max;
                    new_empty_spaces.push_back({right_empty_min,
                                                new_segment_index,
                                                right_empty_max, max_seg_idx});
                }
            }
        }
        empty_spaces = new_empty_spaces;
    }

    void get_final_connected_segments(std::vector<Point_3> &segments_3d) const {
        // std::cout << "Extending segments to fill empty spaces..." <<
        // std::endl; std::cout << "Current segments: " << std::endl; for
        // (std::size_t i = 0; i < current_segments.size(); ++i) {
        //     const auto &[proj_start, proj_end] = current_segments[i];
        //     std::cout << "Segment " << i << " from (" << proj_start.x() << ",
        //     "
        //               << proj_start.y() << ") to (" << proj_end.x() << ", "
        //               << proj_end.y() << ")" << std::endl;
        // }

        std::vector<std::pair<Point_2, Point_2>> extended_segments =
            current_segments;

        // Extend the segments to fill the empty spaces between them
        for (const auto &[empty_left, left_seg_idx, empty_right,
                          right_seg_idx] : empty_spaces) {
            // std::cout << "Empty space from " << empty_left << " to "
            //           << empty_right << " with neighbours ("
            //           << (left_seg_idx.has_value()
            //                   ? std::to_string(left_seg_idx.value())
            //                   : "none")
            //           << ", "
            //           << (right_seg_idx.has_value()
            //                   ? std::to_string(right_seg_idx.value())
            //                   : "none")
            //           << ")" << std::endl;
            if (left_seg_idx.has_value() && right_seg_idx.has_value()) {
                // The empty space is between two segments, we try to see if
                // we can use the intersection between the two

                // std::cout << "Handling empty space between segment "
                //           << left_seg_idx.value() << " and segment "
                //           << right_seg_idx.value() << std::endl;

                auto segment_left = extended_segments[left_seg_idx.value()];
                auto segment_right = extended_segments[right_seg_idx.value()];
                auto result = CGAL::intersection(
                    Line_2(segment_left.first, segment_left.second),
                    Line_2(segment_right.first, segment_right.second));
                if (result) {
                    if (const Point_2 *intersection_point =
                            std::get_if<Point_2>(&*result)) {
                        // The two lines intersect in 2D, but we need to check
                        // if the intersection point is between them on the X
                        // axis

                        if (intersection_point->x() >= empty_left &&
                            intersection_point->x() <= empty_right) {
                            // The intersection point is between the two
                            // segments on the X axis so we can use it

                            // std::cout
                            //     << "Lines intersect and the intersection
                            //     point "
                            //        "is "
                            //        "between the two segments on the X axis, "
                            //        "extending both segments to the "
                            //        "intersection point"
                            //     << std::endl;

                            extended_segments[left_seg_idx.value()].second =
                                *intersection_point;
                            extended_segments[right_seg_idx.value()].first =
                                *intersection_point;
                        } else {
                            // The intersection point is not between the two
                            // segments on the X axis, so we extend both
                            // segments by the same amount to fill the empty
                            // space

                            // std::cout
                            //     << "Lines intersect but the intersection
                            //     point "
                            //        "is not "
                            //        "between the two segments on the X axis, "
                            //        "extending both segments to fill the empty
                            //        " "space"
                            //     << std::endl;

                            double extension_amount =
                                (empty_right - empty_left) / 2;
                            double extension_point_left_x =
                                empty_left + extension_amount;
                            double extension_point_right_x =
                                empty_right - extension_amount;

                            std::pair<Point_2, Point_2> extended_segment_left;
                            std::pair<Point_2, Point_2> extended_segment_right;
                            _extend_segment(
                                segment_left, segment_left.first.x(),
                                extension_point_left_x, extended_segment_left);
                            _extend_segment(segment_right,
                                            extension_point_right_x,
                                            segment_right.second.x(),
                                            extended_segment_right);

                            extended_segments[left_seg_idx.value()] =
                                extended_segment_left;
                            extended_segments[right_seg_idx.value()] =
                                extended_segment_right;
                        }
                    }
                } else {
                    // Otherwise, we extend both by the same amount to fill
                    // the empty space

                    // std::cout << "Lines do not intersect, extending both "
                    //              "segments to fill the empty space"
                    //           << std::endl;

                    double extension_amount = (empty_right - empty_left) / 2;
                    double extension_point_left_x =
                        empty_left + extension_amount;
                    double extension_point_right_x =
                        empty_right - extension_amount;

                    std::pair<Point_2, Point_2> extended_segment_left;
                    std::pair<Point_2, Point_2> extended_segment_right;
                    _extend_segment(segment_left, segment_left.first.x(),
                                    extension_point_left_x,
                                    extended_segment_left);
                    _extend_segment(segment_right, extension_point_right_x,
                                    segment_right.second.x(),
                                    extended_segment_right);

                    extended_segments[left_seg_idx.value()] =
                        extended_segment_left;
                    extended_segments[right_seg_idx.value()] =
                        extended_segment_right;
                }
            } else if (left_seg_idx.has_value()) {
                // The empty space is after the last segment, we extend the
                // last segment to the end of the empty space

                std::pair<Point_2, Point_2> min_segment =
                    extended_segments[left_seg_idx.value()];

                // std::cout << "Extending last segment from ("
                //           << min_segment.first.x() << ", "
                //           << min_segment.first.y() << ") - ("
                //           << min_segment.second.x() << ", "
                //           << min_segment.second.y() << ")" << std::endl;
                std::pair<Point_2, Point_2> extended_segment;
                _extend_segment(min_segment, min_segment.first.x(), empty_right,
                                extended_segment);

                // std::cout << "Extended last segment to ("
                //           << extended_segment.first.x() << ", "
                //           << extended_segment.first.y() << ") - ("
                //           << extended_segment.second.x() << ", "
                //           << extended_segment.second.y() << ")" << std::endl;

                extended_segments[left_seg_idx.value()] = extended_segment;
            } else if (right_seg_idx.has_value()) {
                // The empty space is before the first segment, we extend
                // the first segment to the start of the empty space

                std::pair<Point_2, Point_2> max_segment =
                    extended_segments[right_seg_idx.value()];
                std::pair<Point_2, Point_2> extended_segment;
                _extend_segment(max_segment, empty_left, max_segment.second.x(),
                                extended_segment);

                extended_segments[right_seg_idx.value()] = extended_segment;
            }
        }

        // std::cout << "Done extending segments" << std::endl;

        if (extended_segments.empty()) {
            return;
        }

        // Sort the segments by their minimum X coordinate
        std::sort(extended_segments.begin(), extended_segments.end(),
                  [](const std::pair<Point_2, Point_2> &a,
                     const std::pair<Point_2, Point_2> &b) {
                      double a_min_x = std::min(a.first.x(), a.second.x());
                      double b_min_x = std::min(b.first.x(), b.second.x());
                      return a_min_x < b_min_x;
                  });

        // Export the segments back in 3D in the main space
        // If two consecutive segments are disconnected, we connect them with a
        // vertical segment
        segments_3d.clear();
        segments_3d.reserve(extended_segments.size() + 1);
        for (std::size_t i = 0; i < extended_segments.size(); ++i) {
            const auto &[proj_start, proj_end] = extended_segments[i];
            Point_3 seg_start = origin + horizontal_direction * proj_start.x() +
                                vertical_direction * proj_start.y();
            Point_3 seg_end = origin + horizontal_direction * proj_end.x() +
                              vertical_direction * proj_end.y();
            if (segments_3d.size() > 0) {
                double distance =
                    CGAL::squared_distance(segments_3d.back(), seg_start);
                if (distance > 1e-6) {
                    segments_3d.push_back(seg_start);
                }
            } else {
                segments_3d.push_back(seg_start);
            }
            segments_3d.push_back(seg_end);
        }
    }
};

void SimpleRANSAC3D::get_final_connected_segments(
    std::vector<Point_3> &segments_3d) const {

    Segment2DSpace segment_2d_space(base_segment);

    // Add the segments to the 2D space in order of decreasing score
    for (const RansacSegment3D &segment : best_segments) {
        if (segment.inliers_indices.empty()) {
            std::cerr << "Skipping empty segment inliers" << std::endl;
            continue;
        }

        // Crop the 3D line into a 3D segment based on the first and
        // last inliers
        const std::size_t first_inlier_idx = segment.inliers_indices.front();
        const std::size_t last_inlier_idx = segment.inliers_indices.back();
        const Point_3 &first_inlier = points[first_inlier_idx];
        const Point_3 &last_inlier = points[last_inlier_idx];
        const Point_3 source = segment.line.projection(first_inlier);
        const Point_3 target = segment.line.projection(last_inlier);

        segment_2d_space.add_segment(Segment_3(source, target));
    }

    segment_2d_space.get_final_connected_segments(segments_3d);
}

void one_edge_to_3d(const PtsStructs::StoragePtr &storage,
                    const Segment_2 &edge, std::vector<Point_3> &edge_3d) {
    // std::cout << "Processing edge: " << edge.source() << " - " <<
    // edge.target()
    //           << std::endl;
    const double BUFFER = 0.3;

    // Extract the points in the bounding box of the edge
    const auto kd_tree_2 = storage->get_kd_tree_2d();
    std::vector<std::size_t> point_ids;
    kd_tree_2->search_indices_in_box(edge.bbox(), BUFFER, point_ids);

    // Only select points that are close to the edge in 2D
    std::vector<Point_3> points_3d;
    for (std::size_t i = 0; i < point_ids.size(); ++i) {
        const Point_3 &point =
            storage->get_point(PtsStructs::PointId(point_ids[i]));
        Point_2 point_2d(point.x(), point.y());
        double distance = CGAL::squared_distance(point_2d, edge);
        if (distance < BUFFER * BUFFER) {
            points_3d.push_back(point);
        }
    }

    // If not enough points were found we skip the edge
    if (points_3d.size() < 2) {
        return;
    }
    // std::cout << "Found " << points_3d.size()
    //           << " points in the bounding box of the edge" <<
    //           std::endl;

    // Perform RANSAC in 3D to identify the dominant lines and their
    // inliers
    SimpleRANSAC3D ransac(points_3d, edge, 1000, 0.3);
    ransac.run();
    ransac.get_final_connected_segments(edge_3d);
}

arrow::Status roofprints_to_3d(const std::string &input_roofprints_file,
                               const std::string &edge_points_file,
                               const std::string &output_roofprints_3d_file,
                               bool overwrite) {
    arrow::Status status;

    if (std::filesystem::exists(output_roofprints_3d_file) && !overwrite) {
        throw std::runtime_error("Output file already exists: " +
                                 output_roofprints_3d_file);
    }

    /* ----------------------------------------------------------------------
     */
    /*                           Load the roofprints */
    /* ----------------------------------------------------------------------
     */

    // Read the roofprints data from the Parquet file using the
    // ParquetReader
    ParquetReader roofprints_reader(input_roofprints_file);

    // Optional explicit schema check for the geometry column type.
    std::shared_ptr<arrow::Table> roofprints_table;
    status = roofprints_reader.read_table(roofprints_table);
    if (!status.ok()) {
        std::cerr << "Error reading roofprints table: " << status.ToString()
                  << std::endl;
        return status;
    }

    int geometry_idx = roofprints_table->schema()->GetFieldIndex("geometry");
    if (geometry_idx < 0) {
        return arrow::Status::Invalid(
            "Column 'geometry' not found in roofprints table");
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

        // if (cleabs != "BATIMENT0000000337020997") {
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

    std::cout << "Loaded " << roofprints.size() << " MultiPolygonZ roofprints"
              << std::endl;

    /* ----------------------------------------------------------------------
     */
    /*                          Load the point cloud */
    /* ----------------------------------------------------------------------
     */

    NewLasReader las_reader(edge_points_file);
    auto storage = las_reader.points;
    storage->build_kd_tree_2d();

    /* ----------------------------------------------------------------------
     */
    /*                            Process each edge */
    /* ----------------------------------------------------------------------
     */

    std::cout << "Processing roofprints to extract 3D edges..." << std::endl;
    std::vector<MultiLineStringZWithAttributes> roofprints_3D;
    roofprints_3D.reserve(roofprints.size());
    ProgressBarTotal progress_bar(roofprints.size(), "Processing roofprints");
    progress_bar.start();
    for (const auto &roofprint : roofprints) {
        // std::cout << "Processing roofprint: " << roofprint.get_id()
        //   << std::endl;
        for (int i = 0; i < roofprint.multi_polygon->getNumGeometries(); ++i) {
            // std::cout << "Processing polygon " << i + 1 << "/"
            //           << roofprint.multi_polygon->getNumGeometries()
            //           << std::endl;
            OGRPolygon *polygon = roofprint.multi_polygon->getGeometryRef(i);
            for (int j = 0; j < polygon->getNumInteriorRings() + 1; ++j) {
                OGRMultiLineString *multi_line_string_raw =
                    new OGRMultiLineString();
                OGRLineString *line_string;
                if (j == 0) {
                    line_string = polygon->getExteriorRing();
                } else {
                    line_string = polygon->getInteriorRing(j - 1);
                }

                for (int k = 0; k < line_string->getNumPoints(); ++k) {
                    Point_2 seg_start(line_string->getX(k),
                                      line_string->getY(k));
                    Point_2 seg_end(line_string->getX(
                                        (k + 1) % line_string->getNumPoints()),
                                    line_string->getY(
                                        (k + 1) % line_string->getNumPoints()));
                    Segment_2 edge(seg_start, seg_end);
                    if (std::sqrt(CGAL::squared_distance(seg_start, seg_end)) <
                        0.5) {
                        continue;
                    }
                    std::vector<Point_3> edge_3d;
                    one_edge_to_3d(storage, edge, edge_3d);
                    if (edge_3d.size() == 0) {
                        continue;
                    }
                    if (edge_3d.size() == 1) {
                        std::cerr << "Only one 3D point found for edge from ("
                                  << seg_start.x() << ", " << seg_start.y()
                                  << ") to (" << seg_end.x() << ", "
                                  << seg_end.y() << ")" << std::endl;
                        continue;
                    }
                    OGRLineString line_3d;
                    for (const Point_3 &point_3d : edge_3d) {
                        line_3d.addPoint(point_3d.x(), point_3d.y(),
                                         point_3d.z());
                    }
                    multi_line_string_raw->addGeometry(&line_3d);
                }
                OGRMultiLineStringPtr multi_line_string(multi_line_string_raw);
                roofprints_3D.emplace_back(std::move(multi_line_string),
                                           roofprint.get_id(),
                                           roofprint.get_outline_source());
            }
            progress_bar.increment(1);
        }
    }
    progress_bar.finish();

    /* ----------------------------------------------------------------------
     */
    /*                       Write the output roofprints */
    /* ----------------------------------------------------------------------
     */

    std::filesystem::create_directories(
        std::filesystem::path(output_roofprints_3d_file).parent_path());

    status =
        write_geoms_to_parquet(roofprints_3D, output_roofprints_3d_file, true);
    if (!status.ok()) {
        std::cerr << "Error writing roofprints 3D Parquet file: "
                  << status.ToString() << std::endl;
        return status;
    }

    return status;
}