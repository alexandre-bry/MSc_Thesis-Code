from ..utils.result import Result
from .constants import *
from .geometry import Line, Segment, Vector


def is_reduced_to_point(line: Line, prev_line: Line, next_line: Line) -> bool:
    point_prev = prev_line.intersection_with_line(line).unwrap()
    point_next = next_line.intersection_with_line(line).unwrap()
    return point_prev.distance_to(point_next) < 1e-6


def is_flipped(line: Line, prev_line: Line, next_line: Line) -> bool:
    point_prev = prev_line.intersection_with_line(line).unwrap()
    point_next = next_line.intersection_with_line(line).unwrap()
    return point_prev.to(point_next).dot(line.dir_vector) < 0


def is_problematic(line: Line, prev_line: Line, next_line: Line) -> bool:
    return not is_reduced_to_point(line, prev_line, next_line) and is_flipped(
        line, prev_line, next_line
    )


class AllLines:
    def __init__(
        self,
        lines: list[Line],
        prev_lines: list[int],
        next_lines: list[int],
        touching_lines: list[list[int]],
    ) -> None:
        self.lines = lines
        self.prev_lines = prev_lines
        self.next_lines = next_lines
        self.touching_lines = touching_lines

    def get_indices(self) -> list[int]:
        return list(range(len(self.lines)))

    def get_line(self, idx: int) -> Line:
        return self.lines[idx]

    def update_line(self, idx: int, new_line: Line) -> None:
        self.lines[idx] = new_line

    def shift_line(self, idx: int, shift_vector: Vector) -> None:
        line = self.get_line(idx)
        new_line = line.shifted(shift_vector)
        self.update_line(idx, new_line)

    def get_prev_line_idx(self, idx: int) -> int:
        return self.prev_lines[idx]

    def get_next_line_idx(self, idx: int) -> int:
        return self.next_lines[idx]

    def get_touching_line_indices(self, idx: int) -> list[int]:
        return self.touching_lines[idx]

    def get_segment(self, idx: int) -> Result[Segment, Exception]:
        prev_idx = self.get_prev_line_idx(idx)
        line = self.get_line(idx)
        prev_line = self.get_line(prev_idx)
        segment_result = line.segment(
            prev_line, self.get_line(self.get_next_line_idx(idx))
        )
        if segment_result.is_ok():
            segment_result = Result.ok(
                Segment(
                    segment_result.unwrap().start,
                    segment_result.unwrap().end,
                    name=str(idx),
                )
            )
        return segment_result

    def get_segments(self) -> list[Segment]:
        return [self.get_segment(i).unwrap() for i in range(len(self.lines))]

    def any_problem(self):
        for idx in range(len(self.lines)):
            line = self.get_line(idx)
            prev_line = self.get_line(self.get_prev_line_idx(idx))
            next_line = self.get_line(self.get_next_line_idx(idx))
            if is_problematic(line, prev_line, next_line):
                return True, idx
        return False, None

    def all_problems(self) -> list[int]:
        problematic_indices = []
        for idx in range(len(self.lines)):
            line = self.get_line(idx)
            prev_line = self.get_line(self.get_prev_line_idx(idx))
            next_line = self.get_line(self.get_next_line_idx(idx))
            if is_problematic(line, prev_line, next_line):
                problematic_indices.append(idx)
        return problematic_indices
        return problematic_indices
        return problematic_indices
