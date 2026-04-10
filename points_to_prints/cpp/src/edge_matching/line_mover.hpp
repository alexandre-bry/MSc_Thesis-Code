#include "topology.hpp"
#include <cstddef>

class LineMoverSimple {
  private:
    AllLines::AllOutlinesPtr all_outlines;
    AllLines::EdgeGroupId moving_group_id;
    UnitVector_2 shift_direction;
    std::vector<double> shift_amounts;
    std::vector<std::map<AllLines::EdgeId, double>> computed_shifts;
    std::size_t current_shift_index;
    std::vector<std::tuple<AllLines::EdgeId, bool, bool>> edge_ids_to_check;

    AllLines::Edge get_current_line(AllLines::EdgeId line_id) const;
    void set_current_shift(AllLines::EdgeId line_id, double shift_amount);
    void set_current_shift(AllLines::EdgeGroupId group_id, double shift_amount);
    bool is_currently_shifted(AllLines::EdgeId line_id) const;
    bool has_problem(AllLines::EdgeId focus_line_id) const;
    bool step();

  public:
    LineMoverSimple(AllLines::AllOutlinesPtr _all_outlines,
                    AllLines::EdgeGroupId _moving_group_id,
                    UnitVector_2 _shift_direction,
                    std::vector<double> _shift_amounts);

    void compute_all();
    void get_computed_shifts(
        std::vector<std::map<AllLines::EdgeId, double>> &output) const;
};

class LineMoverSimpleImproved {
  private:
    AllLines::AllOutlinesPtr all_outlines;
    AllLines::EdgeGroupId moving_group_id;
    UnitVector_2 shift_direction;
    std::vector<std::pair<double, AllLines::EdgeId>>
        sorted_shift_thresholds_and_edges;
    std::map<AllLines::EdgeId, double> shift_thresholds;

  public:
    LineMoverSimpleImproved(AllLines::AllOutlinesPtr _all_outlines,
                            AllLines::EdgeGroupId _moving_group_id,
                            UnitVector_2 _shift_direction);

    AllLines::Edge get_line(AllLines::EdgeId line_id,
                            double shift_amount) const;
    void set_threshold(AllLines::EdgeId line_id, double shift_amount);
    void set_threshold(AllLines::EdgeGroupId group_id, double shift_amount);
    void update_sorted_thresholds_and_edges();
    bool has_problem(AllLines::EdgeId focus_line_id) const;

    void compute_steps(double max_shift_amount);
    void get_computed_shifts(double shift_amount,
                             std::map<AllLines::EdgeId, double> &output) const;
};