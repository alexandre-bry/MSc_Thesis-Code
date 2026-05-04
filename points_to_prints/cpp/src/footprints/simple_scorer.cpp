#include "simple_scorer.hpp"

/**
 * A simple scorer that counts positively points in a small buffer around the
 * line and counts negatively points further away in the direction of the
 * translation.
 */
class Scorer {
  protected:
    double distance_threshold;
    double distance_penalty;
    double translation_factor;

  public:
    Scorer(double distance_threshold, double distance_penalty,
           double translation_factor)
        : distance_threshold(distance_threshold),
          distance_penalty(distance_penalty),
          translation_factor(translation_factor) {}

    double compute_score(double signed_horizontal_distance,
                         double translation) const {
        double distance_after_translation =
            signed_horizontal_distance - translation * translation_factor;

        if (std::abs(distance_after_translation) < distance_threshold) {
            return 1.0 -
                   std::abs(distance_after_translation) / distance_threshold;
        } else if (distance_threshold < distance_after_translation &&
                   distance_after_translation <
                       distance_threshold + distance_penalty) {
            // Penalty that is at 0 when the point is at the distance threshold
            // and increases linearly to 1 when the point is at
            // distance_threshold + distance_penalty or more
            double penalty = (distance_after_translation - distance_threshold) /
                             distance_penalty;
            return -penalty;
        } else {
            return 0.0;
        }
    }
};

void score_line_translations(const Line_2 &line,
                             const std::vector<Point_3> &points,
                             const UnitVector_2 &translation_direction,
                             const std::vector<double> &translations,
                             double distance_threshold, double distance_penalty,
                             std::vector<double> &scores) {
    scores.clear();

    // Get the normal direction of the line in the direction of the translation
    // This is in case the translation direction is not perpendicular to the
    // line
    UnitVector_2 line_direction(line.to_vector());
    UnitVector_2 line_normal_direction =
        line_direction.perpendicular(CGAL::COUNTERCLOCKWISE);
    if (line_normal_direction * translation_direction < 0) {
        line_normal_direction = -line_normal_direction;
    }
    double translation_factor = line_normal_direction * translation_direction;

    // Compute the signed horizontal distance from each point to the line in the
    // direction of the translation
    std::vector<double> horizontal_distances;
    for (const auto &point : points) {
        const Point_2 point_2d(point.x(), point.y());
        const Point_2 projected_point = line.projection(point_2d);
        double signed_distance =
            (point_2d - projected_point) * line_normal_direction;
        horizontal_distances.push_back(signed_distance);
    }

    // For each translation, compute a score based on the horizontal distances
    Scorer scorer(distance_threshold, distance_penalty, translation_factor);
    scores.reserve(translations.size());
    for (const auto &translation : translations) {
        double score = 0.0;

        for (const auto &signed_distance : horizontal_distances) {
            score += scorer.compute_score(signed_distance, translation);
        }

        scores.push_back(score);
    }
}