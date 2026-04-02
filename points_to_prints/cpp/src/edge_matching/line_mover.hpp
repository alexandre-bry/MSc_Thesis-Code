#include "topology.hpp"
#include <cstddef>

class LineMoverSimple {
  private:
    AllLines::AllOutlinesPtr all_outlines;
    AllLines::EdgeId moving_line_id;
    AllLines::OptimizationUnitId optim_unit_id;
    Vector_2 shift_direction;
    std::vector<double> shift_amounts;
    std::vector<std::map<AllLines::EdgeId, double>> computed_shifts;
    AllLines::EdgeId prev_further_line_id;
    AllLines::EdgeId next_further_line_id;
    std::size_t current_shift_index;

    AllLines::Edge get_current_line(AllLines::EdgeId line_id) const;
    bool has_problem_main() const;
    bool has_problem_prev() const;
    bool has_problem_next() const;
    bool step() const;

  public:
    LineMoverSimple(AllLines::AllOutlinesPtr _all_outlines,
                    AllLines::EdgeId _moving_line_id, Vector_2 _shift_direction,
                    std::vector<double> _shift_amounts);

    void compute_all();
};