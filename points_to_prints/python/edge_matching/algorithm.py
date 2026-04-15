import logging
import random
from typing import Callable, Dict, List, Tuple

import tqdm

from .constants import *
from .criterion import Criterion
from .geometry import Line, NormalizedVector, Segment
from .line_mover import AllLinesMoverSimple
from .topology import AllLines


class EdgeShiftingAlgorithm:
    def __init__(
        self,
        all_lines: AllLines,
        criterion: Criterion,
    ) -> None:
        self.all_lines = all_lines
        self.criterion = criterion

    def evaluate_lines(self, idxs: list[int], moved_lines: dict[int, Line]) -> float:
        def get_line_local(line_idx: int) -> Line:
            if line_idx in moved_lines:
                return moved_lines[line_idx]
            return self.all_lines.get_line(line_idx)

        segments: List[Segment] = []
        initial_segment_lengths: List[float] = []
        perimeter = 0.0
        for idx in self.all_lines.get_indices():
            prev_line = get_line_local(self.all_lines.get_prev_line_idx(idx))
            line = get_line_local(idx)
            next_line = get_line_local(self.all_lines.get_next_line_idx(idx))

            segment = line.segment(prev_line, next_line).unwrap()

            perimeter += segment.length()
            if idx in idxs:
                segments.append(segment)

                initial_segment_length = self.all_lines.get_initial_edge_length(idx)
                initial_segment_lengths.append(initial_segment_length)

        criterion = self.criterion.evaluate_segments(segments, initial_segment_lengths)

        return criterion

    def optimize_one_line(self, idx: int) -> Tuple[Dict[int, Line], float]:
        line = self.all_lines.get_line(idx)
        shift_direction = line.normal_vector
        shift_direction_flipped = shift_direction.flipped()

        # Prepare all the offsets
        offsets = []
        max_offset_multiplier = int(
            EDGE_MATCHING_OFFSET_ABSOLUTE_MAX / EDGE_MATCHING_OFFSET_STEP
        )
        for offset_multiplier in range(1, max_offset_multiplier + 1):
            offsets.append(offset_multiplier * EDGE_MATCHING_OFFSET_STEP)

        # Compute all the cases
        mover_1 = AllLinesMoverSimple(
            all_lines=self.all_lines,
            line_idx=idx,
            shift_direction=shift_direction,
            queried_shifts=offsets,
        )
        all_shifted_lines_1, all_shifts_1 = mover_1.compute_shifted_lines()
        mover_2 = AllLinesMoverSimple(
            all_lines=self.all_lines,
            line_idx=idx,
            shift_direction=shift_direction_flipped,
            queried_shifts=offsets,
        )
        all_shifted_lines_2, all_shifts_2 = mover_2.compute_shifted_lines()

        all_offsets: List[float] = (
            [-offset for offset in reversed(offsets)] + [0.0] + offsets
        )
        all_shifted_lines: List[Dict[int, Line]] = (
            [{}] + list(reversed(all_shifted_lines_2)) + all_shifted_lines_1
        )
        all_shifts: List[Dict[int, float]] = (
            [{}] + list(reversed(all_shifts_2)) + all_shifts_1
        )
        all_dirs: List[NormalizedVector] = (
            [shift_direction]  # Does not matter
            + [shift_direction_flipped] * len(offsets)
            + [shift_direction] * len(offsets)
        )

        lines_to_consider = set()
        for shifted_lines in all_shifted_lines:
            lines_to_consider.update(shifted_lines.keys())

        # Make sure all the potentially moved lines have their neighbours in the set of lines to consider
        current_lines_to_consider = list(lines_to_consider)
        for line_idx in current_lines_to_consider:
            lines_to_consider.add(self.all_lines.get_prev_line_idx(line_idx))
            lines_to_consider.add(self.all_lines.get_next_line_idx(line_idx))

        # Evaluate the criterion for each offset
        best_criterion_value = float("inf")
        best_offset = 0.0
        best_shift_dir = shift_direction
        best_shifts: Dict[int, Line] = {}
        best_total_shift = 0.0
        for offset, shift_dir, shifted_lines, shifts in tqdm.tqdm(
            zip(all_offsets, all_dirs, all_shifted_lines, all_shifts),
            desc="Evaluating offsets",
            leave=False,
            total=len(all_shifted_lines),
        ):
            criterion_value = self.evaluate_lines(
                list(lines_to_consider), shifted_lines
            )
            if criterion_value < best_criterion_value:
                best_criterion_value = criterion_value
                best_shifts = shifted_lines
                best_offset = offset
                best_shift_dir = shift_dir
                best_total_shift = sum(shifts.values())
        logging.info(
            f"Best offset: {best_offset:.5f} towards direction {best_shift_dir} with criterion value {best_criterion_value:.5f}"
        )

        return best_shifts, best_total_shift

    def optimize_all_lines(
        self,
        callback_after_optimization: Callable[[int], None] = lambda i: None,
    ) -> float:
        # Sort lines by length in descending order
        lengths = [
            self.all_lines.get_segment(i).unwrap().length()
            for i in range(len(self.all_lines.lines))
        ]
        # sorted_indices = sorted(
        #     range(len(lengths)), key=lambda i: lengths[i], reverse=True
        # )
        sorted_indices = list(range(len(self.all_lines.lines)))
        random.shuffle(sorted_indices)

        # Optimize lines in order of their lengths
        sum_shifts = 0.0
        for idx in tqdm.tqdm(sorted_indices, desc="Optimizing lines", leave=False):
            best_shifts, best_total_shift = self.optimize_one_line(idx)
            sum_shifts += best_total_shift
            for line_idx, new_line in best_shifts.items():
                self.all_lines.update_line(line_idx, new_line)
            all_problem_indices = self.all_lines.all_problems()
            if all_problem_indices:
                logging.warning(
                    f"Warning: Self-intersection detected at line indices {all_problem_indices} after optimization of line index {idx}"
                )
            callback_after_optimization(idx)

        return sum_shifts
        return sum_shifts
        return sum_shifts
