#include "simple_scorer.hpp"
#include <cstddef>

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

    double compute_score(const double signed_horizontal_distance,
                         const double translation,
                         const LASclassification::Value classification) const {
        double distance_after_translation =
            signed_horizontal_distance - translation * translation_factor;

        double score = 0.0;

        // Penalty that is at 0 when the point is at 0 and increases
        // linearly to 1 when the point is at distance_penalty in the
        // direction of the translation
        if (0 < distance_after_translation &&
            distance_after_translation < distance_penalty) {
            double penalty_score =
                -distance_after_translation / distance_penalty;
            if (classification == LASclassification::Value::Ground) {
                penalty_score *= 1.0;
            } else {
                penalty_score *= 0.3;
            }
            score += penalty_score;
        }

        // Score that is at 1 when the point is at 0 and decreases linearly
        // to 0 when the point is at distance_threshold on both sides
        if (std::abs(distance_after_translation) < distance_threshold) {
            double point_score =
                1.0 - std::abs(distance_after_translation) / distance_threshold;
            if (classification == LASclassification::Value::Ground) {
                point_score *= 0.3;
            } else {
                point_score *= 1.0;
            }
            score += point_score;
        }

        return score;
    }
};

void score_line_translations(
    const Line_2 &line, const std::vector<Point_3> &points,
    const std::vector<LASclassification::Value> &classifications,
    const UnitVector_2 &translation_direction,
    const std::vector<double> &translations, double distance_threshold,
    double distance_penalty, std::vector<double> &scores) {

    // Check if points and classifications have the same size
    if (points.size() != classifications.size()) {
        throw std::invalid_argument(
            "Points and classifications must have the same size");
    }

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
    for (std::size_t i = 0; i < translations.size(); ++i) {
        double score = 0.0;
        // Print the classifications

        int point_count = 0;
        for (std::size_t j = 0; j < points.size(); ++j) {
            double point_score = scorer.compute_score(
                horizontal_distances[j], translations[i], classifications[j]);
            score += point_score;
            point_count++;
        }

        // std::cout << "Translation: " << translations[i] << ", score: " <<
        // score
        //           << ", point count: " << point_count << std::endl;

        scores.push_back(score);
    }
}