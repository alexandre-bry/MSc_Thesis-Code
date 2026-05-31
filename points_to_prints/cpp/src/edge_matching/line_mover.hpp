#include "topology.hpp"
#include <cstddef>

class LineMoverSimple {
  private:
    const EdgeMatching::AllOutlines *all_outlines;
    EdgeMatching::EdgeGroupId moving_group_id;
    UnitVector_2 shift_direction;
    std::vector<double> shift_amounts;
    std::vector<std::map<EdgeMatching::EdgeId, double>> computed_shifts;
    std::size_t current_shift_index;
    std::vector<std::tuple<EdgeMatching::EdgeId, bool, bool>> edge_ids_to_check;

    EdgeMatching::Edge get_current_line(EdgeMatching::EdgeId line_id) const;
    void set_current_shift(EdgeMatching::EdgeId line_id, double shift_amount);
    void set_current_shift(EdgeMatching::EdgeGroupId group_id,
                           double shift_amount);
    bool is_currently_shifted(EdgeMatching::EdgeId line_id) const;
    bool has_problem(EdgeMatching::EdgeId focus_line_id) const;
    bool step();

  public:
    LineMoverSimple(const EdgeMatching::AllOutlines &_all_outlines,
                    EdgeMatching::EdgeGroupId _moving_group_id,
                    UnitVector_2 _shift_direction,
                    std::vector<double> _shift_amounts);

    void compute_all();
    void get_computed_shifts(
        std::vector<std::map<EdgeMatching::EdgeId, double>> &output) const;
};

class LineMoverSimpleImproved {
  private:
    const EdgeMatching::AllOutlines *all_outlines;
    EdgeMatching::EdgeGroupId moving_group_id;
    UnitVector_2 shift_direction;
    std::vector<std::pair<double, EdgeMatching::EdgeId>>
        sorted_shift_thresholds_and_edges;
    std::map<EdgeMatching::EdgeId, double> shift_thresholds;

  public:
    LineMoverSimpleImproved(const EdgeMatching::AllOutlines &_all_outlines,
                            EdgeMatching::EdgeGroupId _moving_group_id,
                            UnitVector_2 _shift_direction);

    EdgeMatching::Edge get_line(EdgeMatching::EdgeId line_id,
                                double shift_amount) const;
    void set_threshold(EdgeMatching::EdgeId line_id, double shift_amount);
    void set_threshold(EdgeMatching::EdgeGroupId group_id, double shift_amount);
    void update_sorted_thresholds_and_edges();
    bool has_problem(EdgeMatching::EdgeId focus_line_id) const;

    void compute_steps(double max_shift_amount);
    void
    get_computed_shifts(double shift_amount,
                        std::map<EdgeMatching::EdgeId, double> &output) const;
};