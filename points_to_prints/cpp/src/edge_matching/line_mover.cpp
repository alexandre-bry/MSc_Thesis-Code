#include "line_mover.hpp"

#include <CGAL/Distance_3/Point_3_Point_3.h>
#include <cstddef>

#include "../utils/cgal.hpp"
#include "topology.hpp"

const double MIN_EDGE_LENGTH = 1e-6;
const double MIN_EDGE_LENGTH_SQUARED = MIN_EDGE_LENGTH * MIN_EDGE_LENGTH;

bool is_reduced_to_point(const AllLines::Edge &focus_line,
                         const AllLines::Edge &prev_line,
                         const AllLines::Edge &next_line) {
    Point_2 focus_start;
    Point_2 focus_end;
    try {
        focus_start =
            CustomCGAL::intersection(focus_line.to_line(), prev_line.to_line());
        focus_end =
            CustomCGAL::intersection(focus_line.to_line(), next_line.to_line());
    } catch (const std::exception &e) {
        // If the lines are parallel, we consider that the edge is not reduced
        // std::cerr << "Warning: the following lines are parallel, cannot "
        //              "compute intersection:\n"
        //           << "Focus line: " << focus_line.get_initial_start() << " -
        //           "
        //           << focus_line.get_initial_end() << "\n"
        //           << "Previous line: " << prev_line.get_initial_start() << "
        //           - "
        //           << prev_line.get_initial_end() << "\n"
        //           << "Next line: " << next_line.get_initial_start() << " - "
        //           << next_line.get_initial_end() << "\n"
        //           << "Error message: " << e.what() << std::endl;
        return false;
    }

    return CGAL::squared_distance(focus_start, focus_end) <
           MIN_EDGE_LENGTH_SQUARED;
}

bool is_flipped(const AllLines::Edge &focus_line,
                const AllLines::Edge &prev_line,
                const AllLines::Edge &next_line) {
    Point_2 focus_start;
    Point_2 focus_end;
    try {
        focus_start =
            CustomCGAL::intersection(focus_line.to_line(), prev_line.to_line());
        focus_end =
            CustomCGAL::intersection(focus_line.to_line(), next_line.to_line());
    } catch (const std::exception &e) {
        // If the lines are parallel, we consider that the edge is not flipped
        // std::cerr << "Warning: the following lines are parallel, cannot "
        //              "compute intersection:\n"
        //           << "Focus line: " << focus_line.get_initial_start() << " -
        //           "
        //           << focus_line.get_initial_end() << "\n"
        //           << "Previous line: " << prev_line.get_initial_start() << "
        //           - "
        //           << prev_line.get_initial_end() << "\n"
        //           << "Next line: " << next_line.get_initial_start() << " - "
        //           << next_line.get_initial_end() << "\n"
        //           << "Error message: " << e.what() << std::endl;
        return false;
    }

    Vector_2 focus_direction = focus_line.get_direction();

    return focus_direction * (focus_end - focus_start) < 0;
}

bool is_problematic(const AllLines::Edge &focus_line,
                    const AllLines::Edge &prev_line,
                    const AllLines::Edge &next_line) {
    return not is_reduced_to_point(focus_line, prev_line, next_line) &&
           is_flipped(focus_line, prev_line, next_line);
}

LineMoverSimple::LineMoverSimple(AllLines::AllOutlinesPtr _all_outlines,
                                 AllLines::EdgeGroupId _moving_group_id,
                                 UnitVector_2 _shift_direction,
                                 std::vector<double> _shift_amounts)
    : all_outlines(_all_outlines), moving_group_id(_moving_group_id),
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

    // Initialize the values
    this->current_shift_index = 0;
    this->computed_shifts.resize(this->shift_amounts.size());
    this->set_current_shift(this->moving_group_id, 0.0);

    AllLines::EdgeGroup edge_group =
        this->all_outlines->get_edge_group(this->moving_group_id);
    for (const auto &edge_id : edge_group.edge_ids) {
        edge_ids_to_check.push_back(std::make_tuple(edge_id, true, true));
    }

    this->current_shift_index = 1;
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

void LineMoverSimple::set_current_shift(AllLines::EdgeId line_id,
                                        double shift_amount) {
    this->computed_shifts[this->current_shift_index][line_id] = shift_amount;
}

void LineMoverSimple::set_current_shift(AllLines::EdgeGroupId group_id,
                                        double shift_amount) {
    const auto &edge_group = this->all_outlines->get_edge_group(group_id);
    for (const auto &edge_id : edge_group.edge_ids) {
        set_current_shift(edge_id, shift_amount);
    }
}

bool LineMoverSimple::has_problem(AllLines::EdgeId focus_line_id) const {
    AllLines::EdgeId prev_1_line_id =
        this->all_outlines->get_prev_edge_id(focus_line_id);
    AllLines::EdgeId next_1_line_id =
        this->all_outlines->get_next_edge_id(focus_line_id);
    AllLines::EdgeId prev_2_line_id =
        this->all_outlines->get_prev_edge_id(prev_1_line_id);
    AllLines::EdgeId next_2_line_id =
        this->all_outlines->get_next_edge_id(next_1_line_id);

    std::cout << "Checking the following 5 lines in this order: "
              << prev_2_line_id << ", " << prev_1_line_id << ", "
              << focus_line_id << ", " << next_1_line_id << ", "
              << next_2_line_id << std::endl;

    AllLines::Edge focus_line = get_current_line(focus_line_id);
    AllLines::Edge prev_1_line = get_current_line(prev_1_line_id);
    AllLines::Edge next_1_line = get_current_line(next_1_line_id);
    AllLines::Edge prev_2_line = get_current_line(prev_2_line_id);
    AllLines::Edge next_2_line = get_current_line(next_2_line_id);

    return is_problematic(focus_line, prev_1_line, next_1_line) ||
           is_problematic(prev_1_line, prev_2_line, focus_line) ||
           is_problematic(next_1_line, focus_line, next_2_line);
}

bool LineMoverSimple::step() {
    // Leave if all the shift amounts have been processed
    if (this->current_shift_index >= this->shift_amounts.size()) {
        return true;
    }

    auto previous_shift_index = this->current_shift_index - 1;
    double current_shift_amount =
        this->shift_amounts[this->current_shift_index];
    double previous_shift_amount = this->shift_amounts[previous_shift_index];
    double shift_increment = current_shift_amount - previous_shift_amount;

    // Raise an error if the shift increment is negative (should not happen due
    // to checks in the constructor)
    if (shift_increment < 0) {
        throw std::logic_error(
            "Shift amounts must be non-negative and sorted in ascending order");
    }

    // Copy the previous computed shifts to the current index
    for (const auto &entry : this->computed_shifts[previous_shift_index]) {
        set_current_shift(entry.first, current_shift_amount);
    }

    std::set<std::tuple<AllLines::EdgeId, bool, bool>> new_edge_ids_to_check;
    std::set<AllLines::EdgeId> checked_and_moved_edge_ids;

    while (!edge_ids_to_check.empty()) {
        auto [focus_line_id, check_prev, check_next] = edge_ids_to_check.back();
        edge_ids_to_check.pop_back();

        if (checked_and_moved_edge_ids.find(focus_line_id) ==
            checked_and_moved_edge_ids.end()) {
            checked_and_moved_edge_ids.insert(focus_line_id);
        } else {
            continue;
        }

        if (has_problem(focus_line_id)) {
            AllLines::EdgeGroupId focus_line_group_id =
                this->all_outlines->get_edge_group_id(focus_line_id);
            set_current_shift(focus_line_group_id, current_shift_amount);

            // Add all the edges in the same group to the list of edges to check
            const auto &focus_line_group =
                this->all_outlines->get_edge_group(focus_line_group_id);
            for (const auto &edge_id : focus_line_group.edge_ids) {
                if (edge_id != focus_line_id) {
                    edge_ids_to_check.push_back(
                        std::make_tuple(edge_id, true, true));
                }
            }

            // Add previous line if it needs to be checked
            if (check_prev) {
                AllLines::EdgeId prev_line_id =
                    this->all_outlines->get_prev_edge_id(focus_line_id);
                edge_ids_to_check.push_back(
                    std::make_tuple(prev_line_id, true, false));
            }

            // Add next line if it needs to be checked
            if (check_next) {
                AllLines::EdgeId next_line_id =
                    this->all_outlines->get_next_edge_id(focus_line_id);
                edge_ids_to_check.push_back(
                    std::make_tuple(next_line_id, false, true));
            }
        } else {
            new_edge_ids_to_check.insert(
                std::make_tuple(focus_line_id, check_prev, check_next));
        }
    }

    this->edge_ids_to_check =
        std::vector<std::tuple<AllLines::EdgeId, bool, bool>>(
            new_edge_ids_to_check.begin(), new_edge_ids_to_check.end());

    this->current_shift_index++;

    return this->current_shift_index >= this->shift_amounts.size();
}

void LineMoverSimple::compute_all() {
    while (this->current_shift_index < this->shift_amounts.size()) {
        step();
    }
}

void LineMoverSimple::get_computed_shifts(
    std::vector<std::map<AllLines::EdgeId, double>> &output) const {
    output = this->computed_shifts;
}