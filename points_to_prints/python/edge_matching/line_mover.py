import logging
from typing import Callable, Dict, List, Tuple

from .constants import *
from .geometry import Line, UnitVector
from .topology import AllLines, is_problematic, is_reduced_to_point


class AllLinesMoverSimple:
    def __init__(
        self,
        all_lines: AllLines,
        line_idx: int,
        shift_direction: UnitVector,
        queried_shifts: list[float],
    ) -> None:
        self.all_lines = all_lines
        self.main_line_idx = line_idx
        self.shift_direction = shift_direction

        self.all_lines_shifts: List[Dict[int, float]] = [{self.main_line_idx: 0.0}]
        self.iterator_in_shift_ordered = list(range(len(queried_shifts) + 1))
        self.iterator_in_shift_ordered = sorted(
            self.iterator_in_shift_ordered,
            key=lambda i: queried_shifts[i - 1] if i > 0 else 0.0,
        )
        self.queried_shifts = [0.0] + queried_shifts
        if self.queried_shifts[0] < 0:
            raise ValueError("Shift lengths must be non-negative")

        self.prev_further_line_idx = self.all_lines.get_prev_line_idx(
            self.main_line_idx
        )
        self.next_further_line_idx = self.all_lines.get_next_line_idx(
            self.main_line_idx
        )

    def _get_current_line(self, line_idx: int) -> Line:
        current_shift = self.all_lines_shifts[-1].get(line_idx, 0.0)
        return self.all_lines.get_line(line_idx).shifted(
            self.shift_direction * current_shift
        )

    def has_problem_main(self) -> bool:
        focus_line_idx = self.main_line_idx
        prev_line_idx = self.all_lines.get_prev_line_idx(focus_line_idx)
        next_line_idx = self.all_lines.get_next_line_idx(focus_line_idx)
        prev_prev_line_idx = self.all_lines.get_prev_line_idx(prev_line_idx)
        next_next_line_idx = self.all_lines.get_next_line_idx(next_line_idx)

        focus_line = self._get_current_line(focus_line_idx)
        prev_line = self._get_current_line(prev_line_idx)
        next_line = self._get_current_line(next_line_idx)
        prev_prev_line = self._get_current_line(prev_prev_line_idx)
        next_next_line = self._get_current_line(next_next_line_idx)

        problem = (
            is_problematic(line=focus_line, prev_line=prev_line, next_line=next_line)
            or is_problematic(
                line=prev_line, prev_line=prev_prev_line, next_line=focus_line
            )
            or is_problematic(
                line=next_line, prev_line=focus_line, next_line=next_next_line
            )
        )
        return problem

    def has_problem_prev(self) -> bool:
        focus_line_idx = self.prev_further_line_idx
        prev_line_idx = self.all_lines.get_prev_line_idx(focus_line_idx)
        next_line_idx = self.all_lines.get_next_line_idx(focus_line_idx)
        prev_prev_line_idx = self.all_lines.get_prev_line_idx(prev_line_idx)
        next_next_line_idx = self.all_lines.get_next_line_idx(next_line_idx)

        focus_line = self._get_current_line(focus_line_idx)
        prev_line = self._get_current_line(prev_line_idx)
        next_line = self._get_current_line(next_line_idx)
        prev_prev_line = self._get_current_line(prev_prev_line_idx)
        next_next_line = self._get_current_line(next_next_line_idx)

        problem = (
            is_problematic(
                line=next_line, prev_line=focus_line, next_line=next_next_line
            )
            or is_problematic(line=focus_line, prev_line=prev_line, next_line=next_line)
            or is_problematic(
                line=prev_line, prev_line=prev_prev_line, next_line=focus_line
            )
        )
        return problem

    def has_problem_next(self) -> bool:
        focus_line_idx = self.next_further_line_idx
        prev_line_idx = self.all_lines.get_prev_line_idx(focus_line_idx)
        next_line_idx = self.all_lines.get_next_line_idx(focus_line_idx)
        prev_prev_line_idx = self.all_lines.get_prev_line_idx(prev_line_idx)
        next_next_line_idx = self.all_lines.get_next_line_idx(next_line_idx)

        focus_line = self._get_current_line(focus_line_idx)
        prev_line = self._get_current_line(prev_line_idx)
        next_line = self._get_current_line(next_line_idx)
        prev_prev_line = self._get_current_line(prev_prev_line_idx)
        next_next_line = self._get_current_line(next_next_line_idx)

        problem = (
            is_problematic(
                line=prev_line, prev_line=prev_prev_line, next_line=focus_line
            )
            or is_problematic(line=focus_line, prev_line=prev_line, next_line=next_line)
            or is_problematic(
                line=next_line, prev_line=focus_line, next_line=next_next_line
            )
        )
        return problem

    def _compute_next(
        self, callback: Callable[[int, Dict[int, float]], None] = lambda i, d: None
    ) -> bool:
        if len(self.all_lines_shifts) == len(self.queried_shifts):
            return True

        current_shift_index = len(self.all_lines_shifts)
        actual_shift_index = self.iterator_in_shift_ordered[current_shift_index]
        actual_previous_shift_index = self.iterator_in_shift_ordered[
            current_shift_index - 1
        ]
        current_shift = self.queried_shifts[actual_shift_index]
        previous_shift = self.queried_shifts[actual_previous_shift_index]
        shift_to_add = current_shift - previous_shift

        logging.info(
            f"Current shift index: {current_shift_index}, actual shift index: {actual_shift_index}, actual previous shift index: {actual_previous_shift_index}, current shift: {current_shift:.5f}, previous shift: {previous_shift:.5f}, shift to add: {shift_to_add:.5f}"
        )

        if shift_to_add < 0:
            raise ValueError("Shifts should be ordered in increasing order.")

        # Update the shifts for all the lines that are already shifted
        self.all_lines_shifts.append(self.all_lines_shifts[-1].copy())
        for line_idx in self.all_lines_shifts[-1].keys():
            self.all_lines_shifts[-1][line_idx] = current_shift

        if self.has_problem_main():
            logging.info(
                f"Main line ({self.main_line_idx}) has a problem, adding shift to its neighbours {self.prev_further_line_idx} and {self.next_further_line_idx}."
            )
            self.all_lines_shifts[-1][self.prev_further_line_idx] = current_shift
            self.all_lines_shifts[-1][self.next_further_line_idx] = current_shift

            self.prev_further_line_idx = self.all_lines.get_prev_line_idx(
                self.prev_further_line_idx
            )
            self.next_further_line_idx = self.all_lines.get_next_line_idx(
                self.next_further_line_idx
            )

            callback(self.main_line_idx, self.all_lines_shifts[-1])

        while self.has_problem_prev():
            logging.info(
                f"Previous line ({self.prev_further_line_idx}) has a problem, adding shift to its further line."
            )
            self.all_lines_shifts[-1][self.prev_further_line_idx] = current_shift

            callback(self.prev_further_line_idx, self.all_lines_shifts[-1])

            self.prev_further_line_idx = self.all_lines.get_prev_line_idx(
                self.prev_further_line_idx
            )

        while self.has_problem_next():
            logging.info(
                f"Next line ({self.next_further_line_idx}) has a problem, adding shift to its further line."
            )
            self.all_lines_shifts[-1][self.next_further_line_idx] = current_shift

            callback(self.next_further_line_idx, self.all_lines_shifts[-1])

            self.next_further_line_idx = self.all_lines.get_next_line_idx(
                self.next_further_line_idx
            )

        return len(self.all_lines_shifts) == len(self.queried_shifts)

    def compute_lines_shifts(
        self,
        callback_step: Callable[[Dict[int, float]], None] = lambda d: None,
        callback_line: Callable[[int, Dict[int, float]], None] = lambda i, d: None,
    ) -> List[Dict[int, float]]:
        logging.info(
            f"Start computing lines shifts for index {self.main_line_idx} and shift direction {self.shift_direction}."
        )
        while len(self.all_lines_shifts) < len(self.queried_shifts):
            self._compute_next(callback=callback_line)
            callback_step(self.all_lines_shifts[-1])
        ordered_lines_shifts = []
        for shift_index in range(len(self.queried_shifts)):
            actual_shift_index = self.iterator_in_shift_ordered[shift_index]
            ordered_lines_shifts.append(self.all_lines_shifts[actual_shift_index])
        return ordered_lines_shifts

    def compute_shifted_lines(
        self,
        callback_step: Callable[[Dict[int, float]], None] = lambda d: None,
        callback_line: Callable[[int, Dict[int, float]], None] = lambda i, d: None,
    ) -> Tuple[List[Dict[int, Line]], List[Dict[int, float]]]:
        lines_shifts = self.compute_lines_shifts(
            callback_step=callback_step, callback_line=callback_line
        )
        all_shifted_lines = []
        for lines_shift in lines_shifts:
            shifted_lines = {}
            for line_idx, shift in lines_shift.items():
                shifted_lines[line_idx] = self.all_lines.get_line(line_idx).shifted(
                    self.shift_direction * shift
                )
            all_shifted_lines.append(shifted_lines)
        return all_shifted_lines, lines_shifts


def compute_buffer_for_prev(
    next_dominating_line: Line,
    dominating_line: Line,
    focus_line: Line,
    prev_line: Line,
    movement_dir: UnitVector,
) -> Tuple[float, bool]:
    # We consider that the focus line and the next line will move together along the movement direction
    limit = float("inf")
    dominating_line_changed = False

    # 1. Compute the limits with the dominating line itself
    limit_point = focus_line.intersection_with_line(next_dominating_line).unwrap()
    limit_line = Line.from_point_and_dir(limit_point, movement_dir)
    origin_point = limit_line.intersection_with_line(dominating_line).unwrap()
    if origin_point.to(limit_point).dot(movement_dir) > 0:
        new_limit = origin_point.distance_to(limit_point)
        if new_limit < limit:
            logging.info(f"New limit with dominating line: {new_limit:.10f}")
            limit = new_limit
            dominating_line_changed = True

    # 2. Compute the limits with the previous line
    limit_point = focus_line.intersection_with_line(prev_line).unwrap()
    limit_line = Line.from_point_and_dir(limit_point, movement_dir)
    origin_point = limit_line.intersection_with_line(dominating_line).unwrap()
    if origin_point.to(limit_point).dot(movement_dir) > 0:
        new_limit = origin_point.distance_to(limit_point)
        if new_limit < limit:
            logging.info(f"New limit with previous line: {new_limit:.10f}")
            limit = new_limit
            dominating_line_changed = False

    return limit, dominating_line_changed


def compute_buffer_for_next(
    prev_dominating_line: Line,
    dominating_line: Line,
    focus_line: Line,
    next_line: Line,
    movement_dir: UnitVector,
) -> Tuple[float, bool]:
    # We consider that the focus line and the previous line will move together along the movement direction
    limit = float("inf")
    dominating_line_changed = False

    # 1. Compute the limits with the focus line itself
    limit_point = focus_line.intersection_with_line(prev_dominating_line).unwrap()
    limit_line = Line.from_point_and_dir(limit_point, movement_dir)
    origin_point = limit_line.intersection_with_line(dominating_line).unwrap()
    if origin_point.to(limit_point).dot(movement_dir) > 0:
        new_limit = origin_point.distance_to(limit_point)
        if new_limit < limit:
            logging.info(f"New limit with focus line: {new_limit:.10f}")
            limit = new_limit
            dominating_line_changed = True

    # 2. Compute the limits with the next line
    limit_point = focus_line.intersection_with_line(next_line).unwrap()
    logging.info(f"Limit point with next line: {limit_point}")
    limit_line = Line.from_point_and_dir(limit_point, movement_dir)
    origin_point = limit_line.intersection_with_line(dominating_line).unwrap()
    logging.info(f"Origin point with next line: {origin_point}")
    if origin_point.to(limit_point).dot(movement_dir) > 0:
        new_limit = origin_point.distance_to(limit_point)
        if new_limit < limit:
            logging.info(f"New limit with next line: {new_limit:.10f}")
            limit = new_limit
            dominating_line_changed = False

    return limit, dominating_line_changed


def compute_buffer_for_main(
    main_line: Line,
    prev_line: Line,
    next_line: Line,
    prev_prev_line: Line,
    next_next_line: Line,
    movement_dir: UnitVector,
) -> Tuple[float, float, bool]:
    prev_limit = float("inf")
    next_limit = float("inf")
    change_dominating = False

    # 1. Compute the limits with the previous line
    limit_point = prev_line.intersection_with_line(prev_prev_line).unwrap()
    limit_line = Line.from_point_and_dir(limit_point, movement_dir)
    origin_point = limit_line.intersection_with_line(main_line).unwrap()
    if origin_point.to(limit_point).dot(movement_dir) > 0:
        prev_limit = min(prev_limit, origin_point.distance_to(limit_point))

    # 2. Compute the limits with the next line
    limit_point = next_line.intersection_with_line(next_next_line).unwrap()
    limit_line = Line.from_point_and_dir(limit_point, movement_dir)
    origin_point = limit_line.intersection_with_line(main_line).unwrap()
    if origin_point.to(limit_point).dot(movement_dir) > 0:
        next_limit = min(next_limit, origin_point.distance_to(limit_point))

    # 3. Compute the limits with the main line itself
    limit_point = prev_line.intersection_with_line(next_line)
    if limit_point.is_ok():
        limit_point = limit_point.unwrap()
        limit_line = Line.from_point_and_dir(limit_point, movement_dir)
        origin_point = limit_line.intersection_with_line(main_line).unwrap()
        if origin_point.to(limit_point).dot(movement_dir) > 0:
            new_limit = origin_point.distance_to(limit_point)
            if new_limit < prev_limit and new_limit < next_limit:
                prev_limit = new_limit
                next_limit = new_limit
                change_dominating = True

    return prev_limit, next_limit, change_dominating


class AllLinesMoverComplex:
    def __init__(
        self,
        all_lines: AllLines,
        line_idx: int,
        shift_direction: UnitVector,
        queried_shifts: list[float],
    ) -> None:
        self.all_lines = all_lines
        self.main_line_idx = line_idx
        self.shift_direction = shift_direction

        self.lines_shifts: List[Dict[int, float]] = [{self.main_line_idx: 0.0}]
        self.iterator_in_shift_ordered = list(range(len(queried_shifts) + 1))
        self.iterator_in_shift_ordered = sorted(
            self.iterator_in_shift_ordered,
            key=lambda i: queried_shifts[i - 1] if i > 0 else 0.0,
        )
        self.queried_shifts = [0.0] + queried_shifts
        if self.queried_shifts[0] < 0:
            raise ValueError("Shift lengths must be non-negative")

        self.is_reduced_to_point: List[bool] = [False] * (len(self.all_lines.lines) + 1)

        # Initialize the buffers and remember if the changes to the main line
        # change the dominating line
        self.prev_last_buffer, self.next_last_buffer, self.main_changes_dominating = (
            self._compute_current_buffer_for_main(self.main_line_idx)
        )

        _prev_line_idx = self.all_lines.get_prev_line_idx(self.main_line_idx)
        _next_line_idx = self.all_lines.get_next_line_idx(self.main_line_idx)

        self.prev_dominating_line_idx = self.main_line_idx
        self.prev_other_dominating_line_idx = _next_line_idx
        self.prev_further_line_idx = _prev_line_idx
        self.next_dominating_line_idx = self.main_line_idx
        self.next_other_dominating_line_idx = _prev_line_idx
        self.next_further_line_idx = _next_line_idx

        # TODO: Check whether next_other_dominating_line_idx and prev_dominating_line_idx are really different

        # Prepare the future dominating line indices after the topological
        # issue is reached
        if self.main_changes_dominating:
            self.prev_dominating_line_to_change = True
            self.next_dominating_line_to_change = True
        else:
            self.prev_dominating_line_to_change = False
            self.next_dominating_line_to_change = False

    def _add_to_current_shifts(self, line_idx: int, shift_to_add: float) -> None:
        if line_idx in self.lines_shifts[-1]:
            self.lines_shifts[-1][line_idx] += shift_to_add
        else:
            self.lines_shifts[-1][line_idx] = shift_to_add
        logging.info(
            f"Adding shift of {shift_to_add:.5f} to line index {line_idx} in direction {self.shift_direction}"
        )

    def _add_to_all_current_shifts(self, shift_to_add: float) -> None:
        for line_idx in self.lines_shifts[-1].keys():
            if not self.is_reduced_to_point[line_idx]:
                self._add_to_current_shifts(line_idx, shift_to_add)

    def _update_shifts_for_reduced_lines(self) -> None:
        for line_idx in self.lines_shifts[-1].keys():
            if self.is_reduced_to_point[line_idx]:
                prev_line_idx = self.all_lines.get_prev_line_idx(line_idx)
                next_line_idx = self.all_lines.get_next_line_idx(line_idx)
                while self.is_reduced_to_point[prev_line_idx]:
                    prev_line_idx = self.all_lines.get_prev_line_idx(prev_line_idx)
                while self.is_reduced_to_point[next_line_idx]:
                    next_line_idx = self.all_lines.get_next_line_idx(next_line_idx)
                prev_line = self._get_current_line(prev_line_idx)
                next_line = self._get_current_line(next_line_idx)
                objective_point = prev_line.intersection_with_line(next_line).unwrap()
                objective_line = Line.from_point_and_dir(
                    objective_point, self.shift_direction
                )
                current_line = self._get_current_line(line_idx)
                current_point = objective_line.intersection_with_line(
                    current_line
                ).unwrap()
                current_shift_length = current_point.distance_to(objective_point)
                self._add_to_current_shifts(line_idx, current_shift_length)

    def _get_current_line(self, line_idx: int) -> Line:
        current_shift = self.lines_shifts[-1].get(line_idx, 0.0)
        logging.info(f"Getting line {line_idx}: shift {current_shift:.5f}.")
        return self.all_lines.get_line(line_idx).shifted(
            self.shift_direction * current_shift
        )

    def _compute_current_buffer_for_main(
        self, line_idx: int
    ) -> Tuple[float, float, int]:
        prev_idx = self.all_lines.get_prev_line_idx(line_idx)
        next_idx = self.all_lines.get_next_line_idx(line_idx)
        prev_prev_idx = self.all_lines.get_prev_line_idx(prev_idx)
        next_next_idx = self.all_lines.get_next_line_idx(next_idx)

        main_line = self._get_current_line(line_idx)
        prev_line = self._get_current_line(prev_idx)
        next_line = self._get_current_line(next_idx)
        prev_prev_line = self._get_current_line(prev_prev_idx)
        next_next_line = self._get_current_line(next_next_idx)

        return compute_buffer_for_main(
            main_line=main_line,
            prev_line=prev_line,
            next_line=next_line,
            prev_prev_line=prev_prev_line,
            next_next_line=next_next_line,
            movement_dir=self.shift_direction,
        )

    def _compute_current_buffer_for_prev(self) -> Tuple[float, bool]:
        focus_line_idx = self.prev_further_line_idx
        prev_line_idx = self.all_lines.get_prev_line_idx(focus_line_idx)

        logging.info(
            f"Computing previous buffer with focus line index {focus_line_idx}, previous line index {prev_line_idx}, dominating line index {self.prev_dominating_line_idx} and other dominating line index {self.prev_other_dominating_line_idx}."
        )

        # _next_line_idx = self.all_lines.get_next_line_idx(focus_line_idx)
        # if self.is_reduced_to_point[_next_line_idx]:
        #     return 0.0, False

        focus_line = self._get_current_line(focus_line_idx)
        prev_line = self._get_current_line(prev_line_idx)
        dominating_line = self._get_current_line(self.prev_dominating_line_idx)
        other_dominating_line = self._get_current_line(
            self.prev_other_dominating_line_idx
        )

        buffer, dominating_line_changed = compute_buffer_for_prev(
            dominating_line=dominating_line,
            next_dominating_line=other_dominating_line,
            focus_line=focus_line,
            prev_line=prev_line,
            movement_dir=self.shift_direction,
        )
        return buffer, dominating_line_changed

    def _compute_current_buffer_for_next(self) -> Tuple[float, bool]:
        focus_line_idx = self.next_further_line_idx
        next_line_idx = self.all_lines.get_next_line_idx(focus_line_idx)

        logging.info(
            f"Computing next buffer with focus line index {focus_line_idx}, next line index {next_line_idx}, dominating line index {self.next_dominating_line_idx} and other dominating line index {self.next_other_dominating_line_idx}."
        )

        # _prev_line_idx = self.all_lines.get_prev_line_idx(focus_line_idx)
        # if self.is_reduced_to_point[_prev_line_idx]:
        #     return 0.0, False

        focus_line = self._get_current_line(focus_line_idx)
        next_line = self._get_current_line(next_line_idx)
        dominating_line = self._get_current_line(self.next_dominating_line_idx)
        other_dominating_line = self._get_current_line(
            self.next_other_dominating_line_idx
        )

        buffer, dominating_line_changed = compute_buffer_for_next(
            dominating_line=dominating_line,
            prev_dominating_line=other_dominating_line,
            focus_line=focus_line,
            next_line=next_line,
            movement_dir=self.shift_direction,
        )

        return buffer, dominating_line_changed

    def _compute_next(self) -> bool:
        if len(self.lines_shifts) == len(self.queried_shifts):
            return True

        current_shift_index = len(self.lines_shifts)
        actual_shift_index = self.iterator_in_shift_ordered[current_shift_index]
        actual_previous_shift_index = self.iterator_in_shift_ordered[
            current_shift_index - 1
        ]
        current_shift = self.queried_shifts[actual_shift_index]
        previous_shift = self.queried_shifts[actual_previous_shift_index]
        shift_to_add = current_shift - previous_shift

        logging.info(
            f"Current shift index: {current_shift_index}, actual shift index: {actual_shift_index}, actual previous shift index: {actual_previous_shift_index}, current shift: {current_shift:.5f}, previous shift: {previous_shift:.5f}, shift to add: {shift_to_add:.5f}"
        )

        if shift_to_add < 0:
            raise ValueError("Shifts should be ordered in increasing order.")

        # # Situation where the shift is not large enough to cause any topological change
        # if shift_to_add < min(self.prev_last_buffer, self.next_last_buffer):
        #     # Update the buffers by simply subtracting the shift to add
        #     self.prev_last_buffer -= shift_to_add
        #     self.next_last_buffer -= shift_to_add

        #     # Simply add the shift to all the lines that are already shifted
        #     new_shifts = {
        #         line_idx: shift + shift_to_add
        #         for line_idx, shift in self.lines_shifts[-1].items()
        #     }
        #     self.lines_shifts.append(new_shifts)
        #     return len(self.lines_shifts) == len(self.queried_shifts)

        # Otherwise, at least one of the buffers will be fully consumed
        current_shifts = self.lines_shifts[-1]
        self.lines_shifts.append(current_shifts.copy())
        while True:
            logging.info(f"Still processing line {self.main_line_idx}.")
            # Situation where the shift is not large enough to cause any topological change
            if shift_to_add < min(self.prev_last_buffer, self.next_last_buffer):
                logging.info(
                    f"Shift to add ({shift_to_add:.5f}) is smaller than both buffers (prev: {self.prev_last_buffer:.5f}, next: {self.next_last_buffer:.5f}), simply adding the shift to all the currently shifted lines."
                )
                # Update the buffers by simply subtracting the shift to add
                self.prev_last_buffer -= shift_to_add
                self.next_last_buffer -= shift_to_add

                # Simply add the shift to all the lines that are already shifted
                self._add_to_all_current_shifts(shift_to_add)
                self._update_shifts_for_reduced_lines()
                return len(self.lines_shifts) == len(self.queried_shifts)

            # Update the lines reduced to point
            for line_idx in range(len(self.all_lines.lines)):
                prev_line_idx = self.all_lines.get_prev_line_idx(line_idx)
                next_line_idx = self.all_lines.get_next_line_idx(line_idx)

                line = self._get_current_line(line_idx)
                prev_line = self._get_current_line(prev_line_idx)
                next_line = self._get_current_line(next_line_idx)

                if is_reduced_to_point(
                    line=line, prev_line=prev_line, next_line=next_line
                ):
                    self.is_reduced_to_point[line_idx] = True
                else:
                    self.is_reduced_to_point[line_idx] = False

            if self.main_changes_dominating:
                logging.info("Main line changes dominating line.")
                self.main_changes_dominating = False
                # Special case of the main line which ends because it reaches
                # the intersection between itw two neighbours
                buffer_to_add = self.prev_last_buffer  # equal to self.next_last_buffer
                self.prev_last_buffer = 0.0
                self.next_last_buffer = 0.0
                shift_to_add -= buffer_to_add

                self._add_to_current_shifts(self.main_line_idx, buffer_to_add)
                self._add_to_current_shifts(self.prev_further_line_idx, 0.0)
                self._add_to_current_shifts(self.next_further_line_idx, 0.0)

                self.is_reduced_to_point[self.main_line_idx] = True

                # Update the lines indices
                if self.prev_dominating_line_to_change:
                    self.prev_dominating_line_idx = self.prev_further_line_idx
                self.prev_other_dominating_line_idx = self.next_further_line_idx
                if self.next_dominating_line_to_change:
                    self.next_dominating_line_idx = self.next_further_line_idx
                self.next_other_dominating_line_idx = self.prev_further_line_idx

                self.prev_further_line_idx = self.all_lines.get_prev_line_idx(
                    self.prev_further_line_idx
                )
                self.next_further_line_idx = self.all_lines.get_next_line_idx(
                    self.next_further_line_idx
                )

                # Compute the new buffer for the previous line
                buffer, dominating_line_to_change = (
                    self._compute_current_buffer_for_prev()
                )
                self.prev_last_buffer = buffer
                self.prev_dominating_line_to_change = dominating_line_to_change

                # Compute the new buffer for the next line
                buffer, dominating_line_to_change = (
                    self._compute_current_buffer_for_next()
                )
                self.next_last_buffer = buffer
                self.next_dominating_line_to_change = dominating_line_to_change

                self._update_shifts_for_reduced_lines()

            else:
                if self.prev_last_buffer < self.next_last_buffer:
                    logging.info(
                        f"Previous line ({self.prev_further_line_idx}) reaches its buffer first: {self.prev_last_buffer:.5f} < {self.next_last_buffer:.5f}."
                    )
                    focus_index = self.prev_further_line_idx
                    initial_dominating_line_idx = self.prev_dominating_line_idx

                    # Look at the previous side
                    buffer_to_add = self.prev_last_buffer
                    self.prev_last_buffer = 0.0
                    self.next_last_buffer -= buffer_to_add
                    shift_to_add -= buffer_to_add

                    # Update the shifts by adding the buffer to add to all the
                    # currently shifted lines and adding the new line to shift
                    self._add_to_all_current_shifts(buffer_to_add)
                    self._add_to_current_shifts(self.prev_further_line_idx, 0.0)

                    # Update the lines
                    if self.prev_dominating_line_to_change:
                        self.prev_dominating_line_idx = self.prev_further_line_idx
                    self.next_other_dominating_line_idx = self.prev_dominating_line_idx
                    self.prev_further_line_idx = self.all_lines.get_prev_line_idx(
                        self.prev_further_line_idx
                    )

                    if self.prev_dominating_line_to_change:
                        reduced_to_point_idx = initial_dominating_line_idx
                    else:
                        reduced_to_point_idx = focus_index
                    self.is_reduced_to_point[reduced_to_point_idx] = True

                    # Compute the new buffer for the previous line
                    buffer, dominating_line_to_change = (
                        self._compute_current_buffer_for_prev()
                    )
                    self.prev_last_buffer = buffer
                    self.prev_dominating_line_to_change = dominating_line_to_change

                    self._update_shifts_for_reduced_lines()

                    logging.info(
                        f"New previous line buffer: {self.prev_last_buffer:.5f}, reduced to point: {reduced_to_point_idx}"
                    )

                else:
                    logging.info(
                        f"Next line ({self.next_further_line_idx}) reaches its buffer first: {self.next_last_buffer:.5f} < {self.prev_last_buffer:.5f}."
                    )
                    focus_index = self.next_further_line_idx
                    initial_dominating_line_idx = self.next_dominating_line_idx

                    # Look at the next side
                    buffer_to_add = self.next_last_buffer
                    self.next_last_buffer = 0.0
                    self.prev_last_buffer -= buffer_to_add
                    shift_to_add -= buffer_to_add

                    # Update the shifts by adding the buffer to add to all the
                    # currently shifted lines and adding the new line to shift
                    self._add_to_all_current_shifts(buffer_to_add)
                    self._add_to_current_shifts(self.next_further_line_idx, 0.0)

                    # Update the lines
                    if self.next_dominating_line_to_change:
                        self.next_dominating_line_idx = self.next_further_line_idx
                    self.prev_other_dominating_line_idx = self.next_dominating_line_idx
                    self.next_further_line_idx = self.all_lines.get_next_line_idx(
                        self.next_further_line_idx
                    )

                    if self.next_dominating_line_to_change:
                        reduced_to_point_idx = initial_dominating_line_idx
                    else:
                        reduced_to_point_idx = focus_index
                    self.is_reduced_to_point[reduced_to_point_idx] = True

                    # Compute the new buffer for the next line
                    buffer, dominating_line_to_change = (
                        self._compute_current_buffer_for_next()
                    )
                    self.next_last_buffer = buffer
                    self.next_dominating_line_to_change = dominating_line_to_change

                    self._update_shifts_for_reduced_lines()

                    logging.info(
                        f"New next line buffer: {self.next_last_buffer:.5f}, reduced to point: {reduced_to_point_idx}"
                    )

    def compute_lines_shifts(self) -> List[Dict[int, float]]:
        logging.info(
            f"Start computing lines shifts for index {self.main_line_idx} and shift direction {self.shift_direction}."
        )
        shift_lengths = sorted(self.queried_shifts)
        while len(self.lines_shifts) < len(shift_lengths):
            self._compute_next()
        ordered_lines_shifts = []
        for shift_index in range(len(shift_lengths)):
            actual_shift_index = self.iterator_in_shift_ordered[shift_index]
            ordered_lines_shifts.append(self.lines_shifts[actual_shift_index])
        return ordered_lines_shifts

    def compute_shifted_lines(self) -> List[Dict[int, Line]]:
        lines_shifts = self.compute_lines_shifts()
        all_shifted_lines = []
        for lines_shift in lines_shifts:
            shifted_lines = {}
            for line_idx, shift in lines_shift.items():
                shifted_lines[line_idx] = self.all_lines.get_line(line_idx).shifted(
                    self.shift_direction * shift
                )
            all_shifted_lines.append(shifted_lines)
        return all_shifted_lines
        return all_shifted_lines
