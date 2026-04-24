#include <cstddef>
#include <random>
#include <vector>

#include "../las/reader.hpp"
#include "../parquet.hpp"
#include "../parquet/reader.hpp"
#include "../points.hpp"
#include "../utils/cgal.hpp"

struct RansacSegment3D {
    Line_3 line;
    std::vector<std::size_t> inliers_indices;
    std::vector<double> inliers_scores;
    double total_score;
    double min_projection_value;
    double max_projection_value;

    RansacSegment3D(const Line_3 &line,
                    const std::vector<std::size_t> &inliers_indices,
                    const std::vector<double> &inliers_scores,
                    double min_projection_value, double max_projection_value)
        : line(line), inliers_indices(inliers_indices),
          inliers_scores(inliers_scores),
          min_projection_value(min_projection_value),
          max_projection_value(max_projection_value) {
        total_score = 0.0;
        for (double score : inliers_scores) {
            total_score += score;
        }
    }
};

class SimpleRANSAC3D {
  private:
    const Segment_2 base_segment;
    int samples;
    double distance_threshold;

    std::random_device rd;
    std::mt19937 gen;

    std::vector<Point_3> points; // The points to process, sorted by their
                                 // projection on the base segment
    std::vector<double> points_projection_value;

    std::vector<RansacSegment3D> best_segments;

    /**
     * @brief Checks whether the segment of projection value spanned by the
     * given segment is large enough after removing the parts that are
     * already covered by the selected segments.
     *
     * @param s1
     * @param selected_segments
     * @return true
     * @return false
     */
    bool _is_different_enough(
        const RansacSegment3D &s1,
        const std::vector<RansacSegment3D> &selected_segments) const;

    /**
     * @brief Computes a similarity score between two segments based on the
     * number of inliers they have in common. It is computed as the number of
     * shared inliers divided by the minimum number of inliers between the two
     * segments.
     *
     * @param s1 First segment
     * @param s2 Second segment
     * @return double Similarity score between 0 and 1, where 1 means that the
     * segments have the same inliers and 0 means that they have no inliers in
     * common.
     */
    double _similarity_inliers(const RansacSegment3D &s1,
                               const RansacSegment3D &s2) const;

    /**
     * @brief Compute the segment that optimizes the inliers of the given
     * segment
     *
     * @param segment
     * @return double
     */
    double _optimize_segment(const RansacSegment3D &segment) const;

  public:
    SimpleRANSAC3D(const std::vector<Point_3> &_points,
                   const Segment_2 &_base_segment, int _samples,
                   double _distance_threshold);

    void process_line(const Line_3 &line);
    void run();
    void get_final_segments(std::vector<Segment_3> &segments_3d) const;
};

void one_edge_to_3d(const PtsStructs::StoragePtr &storage,
                    const Segment_2 &edge, std::vector<Segment_3> &edge_3d);

struct RooprintsTo3DOptions {
    std::string input_roofprints_file;
    std::string edge_points_file;
    std::string output_roofprints_3d_file;
    bool overwrite;
};

arrow::Status roofprints_to_3d(const std::string &input_roofprints_file,
                               const std::string &edge_points_file,
                               const std::string &output_roofprints_3d_file,
                               bool overwrite);