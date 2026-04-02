#include "line_mover.hpp"

#include <cstddef>

#include "../utils/cgal.hpp"
#include "topology.hpp"

LineMoverSimple::LineMoverSimple(AllLines::AllOutlinesPtr _all_outlines,
                                 AllLines::EdgeId _moving_line_id,
                                 Vector_2 _shift_direction,
                                 std::vector<double> _shift_amounts)
    : all_outlines(_all_outlines), moving_line_id(_moving_line_id),
      shift_direction(_shift_direction), shift_amounts(_shift_amounts) {

    // Normalize the shift direction
    this->shift_direction = this->shift_direction /
                            std::sqrt(this->shift_direction.squared_length());

    // Ensure that the shift amounts are non-negative and sorted in
    // ascending order
    for (std::size_t i = 0; i < this->shift_amounts.size(); ++i) {
        if (this->shift_amounts[i] < 0) {
            throw std::invalid_argument("Shift amounts must be non-negative");
        }
        if (i > 0 && this->shift_amounts[i] < this->shift_amounts[i - 1]) {
            throw std::invalid_argument(
                "Shift amounts must be sorted in ascending order");
        }
    }

    // Add a zero shift amount at the beginning to have initial values
    this->shift_amounts =
        std::vector<double>(this->shift_amounts.size() + 1, 0.0);
    std::copy(_shift_amounts.begin(), _shift_amounts.end(),
              this->shift_amounts.begin() + 1);

    this->optim_unit_id = this->all_outlines->get_optim_unit_id(
        this->all_outlines->get_edge_group_id(this->moving_line_id));

    // Initialize the values
    this->current_shift_index = 0;
    this->computed_shifts.resize(this->shift_amounts.size());
    this->computed_shifts[0][this->moving_line_id] = 0.0;

    this->prev_further_line_id =
        this->all_outlines->get_prev_edge_id(this->moving_line_id);
    this->next_further_line_id =
        this->all_outlines->get_next_edge_id(this->moving_line_id);
}

AllLines::Edge
LineMoverSimple::get_current_line(AllLines::EdgeId line_id) const {
    if (this->computed_shifts[this->current_shift_index].count(line_id) == 0) {
        return this->all_outlines->get_edge(line_id);
    } else {
        double shift_amount =
            this->computed_shifts[this->current_shift_index].at(line_id);
        Vector_2 shift_vector = shift_amount * this->shift_direction;
        return this->all_outlines->get_edge(line_id).translated(shift_vector);
    }
}
bool LineMoverSimple::has_problem_main() const {}
bool LineMoverSimple::has_problem_prev() const {}
bool LineMoverSimple::has_problem_next() const {}
bool LineMoverSimple::step() const {}
void LineMoverSimple::compute_all() {}