import logging
from typing import Callable, Dict, List, Optional, Tuple

from ..utils.result import Result
from .constants import *
from .geometry import Line, Point, Segment, UnitVector, Vector
from .topology import is_flipped, is_problematic, is_reduced_to_point


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


def handle_self_intersection(
    line_init: "Line",
    line: "Line",
    prev_prev_line: "Line",
    prev_line: "Line",
    next_line: "Line",
    next_next_line: "Line",
    prev_movable: bool,
    next_movable: bool,
) -> Tuple[Tuple[Optional[Point], bool], Tuple[Optional[Point], bool]]:
    # TODO: Fix this function
    # Maybe an idea would be to think about all possible cases
    # I think that handling everything in one function would make it easier, as it would allow to go all the way to the end of the chain in one go

    if not prev_movable and not next_movable:
        raise ValueError(
            "Both neighbours are not movable, cannot resolve self-intersection"
        )

    # Check for a self-intersection between the two neighbours
    point_prev_prev = prev_prev_line.intersection_with_line(prev_line).unwrap()
    point_prev = prev_line.intersection_with_line(line).unwrap()
    point_next = line.intersection_with_line(next_line).unwrap()
    point_next_next = next_line.intersection_with_line(next_next_line).unwrap()

    # Check whether the main edge was flipped
    if point_prev.to(point_next).dot(line.dir_vector) < 0:
        if prev_movable and next_movable:
            intersection_point = prev_line.intersection_with_line(next_line).unwrap()
            intersection_point_projected = line.projection_on_line(intersection_point)
            return (intersection_point_projected, True), (
                intersection_point_projected,
                True,
            )
        elif prev_movable:
            intersection_point = next_line.intersection_with_line(line).unwrap()
            return (intersection_point, True), (None, False)
        else:
            intersection_point = prev_line.intersection_with_line(line).unwrap()
            return (None, False), (intersection_point, True)

    # Check whether neighbour edges were flipped
    intersection_prev: Optional[Point] = None
    continue_prev = False
    intersection_next: Optional[Point] = None
    continue_next = False
    prev_prev_to_prev = point_prev_prev.to(point_prev)

    if prev_prev_to_prev.length() < 1e-10:
        return (None, True), (None, False)
    elif prev_prev_to_prev.dot(prev_line.dir_vector) < 0:
        if prev_movable:
            intersection_prev = prev_prev_line.intersection_with_line(line).unwrap()
            continue_prev = True
        else:
            logging.debug(
                "Previous neighbour is not movable, cannot resolve self-intersection"
            )
            logging.debug(f"{point_prev_prev.to(point_prev)=}")
            # fig, ax = plt.subplots()
            # lines = {
            #     "black": (
            #         line_init,
            #         line_init.intersection_with_line(prev_line).unwrap(),
            #     ),
            #     "red": (prev_prev_line, point_prev_prev),
            #     "orange": (prev_line, point_prev),
            #     "green": (line, point_next),
            #     "blue": (next_line, point_next),
            #     "purple": (next_next_line, point_next_next),
            # }
            # for color, (line, arrow_position) in lines.items():
            #     line.plot(ax=ax, color=color, arrow_position=arrow_position)
            # points = {
            #     "red": point_prev_prev,
            #     "orange": point_prev,
            #     "blue": point_next,
            #     "purple": point_next_next,
            # }
            # for color, point in points.items():
            #     point.plot(ax=ax, color=color, size=10)
            # plt.show()
            # raise ValueError(
            #     "Previous neighbour is not movable, cannot resolve self-intersection"
            # )

    next_to_next_next = point_next.to(point_next_next)
    if next_to_next_next.length() < 1e-10:
        return (None, False), (None, True)
    elif next_to_next_next.dot(next_line.dir_vector) < 0:
        if next_movable:
            intersection_next = next_next_line.intersection_with_line(line).unwrap()
            continue_next = True
        else:
            logging.debug(
                "Next neighbour is not movable, cannot resolve self-intersection"
            )
            logging.debug(f"{point_next.to(point_next_next)=}")
            # fig, ax = plt.subplots()
            # lines = {
            #     "black": line_init,
            #     "red": prev_prev_line,
            #     "orange": prev_line,
            #     "green": line,
            #     "blue": next_line,
            #     "purple": next_next_line,
            # }
            # for color, line in lines.items():
            #     line.plot(ax=ax, color=color)
            # points = {
            #     "red": point_prev_prev,
            #     "orange": point_prev,
            #     "blue": point_next,
            #     "purple": point_next_next,
            # }
            # for color, point in points.items():
            #     point.plot(ax=ax, color=color, size=10)
            # plt.show()
            # raise ValueError(
            #     "Next neighbour is not movable, cannot resolve self-intersection"
            # )

    return (intersection_prev, continue_prev), (intersection_next, continue_next)


class Buffer:

    def __init__(
        self,
        forward_pref: float,
        backward_prev: float,
        forward_next: float,
        backward_next: float,
    ) -> None:
        self.forward_prev = forward_pref
        self.backward_prev = backward_prev
        self.forward_next = forward_next
        self.backward_next = backward_next

    def __str__(self) -> str:
        return f"Buffer(forward_prev={self.forward_prev:.10f}, backward_prev={self.backward_prev:.10f}, forward_next={self.forward_next:.10f}, backward_next={self.backward_next:.10f})"

    def __repr__(self) -> str:
        return self.__str__()


def compute_buffer_self_moving(
    focus_line: Line,
    prev_line: Line,
    next_line: Line,
    prev_prev_line: Line,
    next_next_line: Line,
    movement_dir: UnitVector,
) -> Buffer:
    """Computes the limits of the movement of the focus line in the direction of the movement vector, in both directions.
    Takes into account:
    - Not flipping the previous line
    - Not flipping the next line
    - Not flipping the focus line

    Args:
        focus_line (Line): The line that we want to move.
        prev_line (Line): The previous line.
        next_line (Line): The next line.
        prev_prev_line (Line): The previous previous line.
        next_next_line (Line): The next next line.
        movement_dir (NormalizedVector): The direction of the movement.

    Returns:
        Buffer: The limits of the movement.
    """
    buffer = Buffer(
        forward_pref=float("inf"),
        backward_prev=float("inf"),
        forward_next=float("inf"),
        backward_next=float("inf"),
    )
    # 1. Compute the limits with the previous line
    limit_point = prev_line.intersection_with_line(prev_prev_line).unwrap()
    limit_line = Line.from_point_and_dir(limit_point, movement_dir)
    origin_point = limit_line.intersection_with_line(focus_line).unwrap()
    if origin_point.to(limit_point).dot(movement_dir) > 0:
        buffer.forward_prev = min(
            buffer.forward_prev, origin_point.distance_to(limit_point)
        )
    else:
        buffer.backward_prev = min(
            buffer.backward_prev, origin_point.distance_to(limit_point)
        )

    # 2. Compute the limits with the next line
    limit_point = next_line.intersection_with_line(next_next_line).unwrap()
    limit_line = Line.from_point_and_dir(limit_point, movement_dir)
    origin_point = limit_line.intersection_with_line(focus_line).unwrap()
    if origin_point.to(limit_point).dot(movement_dir) > 0:
        buffer.forward_next = min(
            buffer.forward_next, origin_point.distance_to(limit_point)
        )
    else:
        buffer.backward_next = min(
            buffer.backward_next, origin_point.distance_to(limit_point)
        )

    # 3. Compute the limits with the focus line itself
    limit_point = prev_line.intersection_with_line(next_line)
    if limit_point.is_ok():
        limit_point = limit_point.unwrap()
        limit_line = Line.from_point_and_dir(limit_point, movement_dir)
        origin_point = limit_line.intersection_with_line(focus_line).unwrap()
        if origin_point.to(limit_point).dot(movement_dir) > 0:
            buffer.forward_prev = min(
                buffer.forward_prev, origin_point.distance_to(limit_point)
            )
            buffer.forward_next = min(
                buffer.forward_next, origin_point.distance_to(limit_point)
            )
        else:
            buffer.backward_prev = min(
                buffer.backward_prev, origin_point.distance_to(limit_point)
            )
            buffer.backward_next = min(
                buffer.backward_next, origin_point.distance_to(limit_point)
            )

    return buffer


def compute_buffer_prev_moving(
    focus_line: Line,
    prev_line: Line,
    next_line: Line,
    prev_prev_line: Line,
    next_next_line: Line,
    movement_dir: UnitVector,
) -> Tuple[float, float]:
    """Computes the limit of the movement of previous line in the direction of the movement vector, in both directions.
    Takes into account:
    - Not flipping the previous line
    - Not flipping the next line
    - Not flipping the focus line

    Args:
        focus_line (Line): The line that we want to compute the buffer for.
        prev_line (Line): The previous line (which is the one moving).
        next_line (Line): The next line.
        prev_prev_line (Line): The previous previous line.
        next_next_line (Line): The next next line.
        movement_dir (NormalizedVector): The direction of the movement.

    Returns:
        Tuple[float, float]: The limits of the movement, first in the direction of the movement vector, then in the opposite direction.
    """
    limit, limit_opposite = float("inf"), float("inf")
    # 1. Compute the limits with the focus line
    limit_point = focus_line.intersection_with_line(next_line).unwrap()
    limit_line = Line.from_point_and_dir(limit_point, movement_dir)
    origin_point = limit_line.intersection_with_line(prev_line).unwrap()
    if origin_point.to(limit_point).dot(movement_dir) > 0:
        limit = min(limit, origin_point.distance_to(limit_point))
    else:
        limit_opposite = min(limit_opposite, origin_point.distance_to(limit_point))

    # 2. Compute the limits with the previous previous line
    limit_point = prev_prev_line.intersection_with_line(prev_line).unwrap()
    limit_line = Line.from_point_and_dir(limit_point, movement_dir)
    origin_point = limit_line.intersection_with_line(prev_line).unwrap()
    if origin_point.to(limit_point).dot(movement_dir) > 0:
        limit = min(limit, origin_point.distance_to(limit_point))
    else:
        limit_opposite = min(limit_opposite, origin_point.distance_to(limit_point))

    return limit, limit_opposite


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

    def _old_lines_to_move_if_line_is_shifted(
        self, idx: int, shift: Vector
    ) -> Dict[int, Line]:
        shifted_lines = {idx: self.get_line(idx).shifted(shift)}
        next_lines_to_process: List[Tuple[int, bool, bool]] = [(idx, True, True)]

        def get_line_local(line_idx: int) -> Line:
            if line_idx in shifted_lines:
                return shifted_lines[line_idx]
            return self.get_line(line_idx)

        def check_self_intersection(
            line_idx: int,
            prev_movable: bool,
            next_movable: bool,
        ) -> Tuple[Tuple[Optional[Point], bool], Tuple[Optional[Point], bool]]:
            prev_idx = self.get_prev_line_idx(line_idx)
            next_idx = self.get_next_line_idx(line_idx)
            prev_prev_idx = self.get_prev_line_idx(prev_idx)
            next_next_idx = self.get_next_line_idx(next_idx)

            prev_prev_line = get_line_local(prev_prev_idx)
            prev_line = get_line_local(prev_idx)
            line_shifted = get_line_local(line_idx)
            next_line = get_line_local(next_idx)
            next_next_line = get_line_local(next_next_idx)

            return handle_self_intersection(
                line_init=self.get_line(line_idx),
                line=line_shifted,
                prev_prev_line=prev_prev_line,
                prev_line=prev_line,
                next_line=next_line,
                next_next_line=next_next_line,
                prev_movable=prev_movable,
                next_movable=next_movable,
            )

        while len(next_lines_to_process) > 0:
            # logging.debug(f"Processing line index {not_processed_lines[0]}")
            current_idx, prev_movable, next_movable = next_lines_to_process.pop(0)
            # Check self-intersection of the current line
            (intersection_prev, continue_prev), (intersection_next, continue_next) = (
                check_self_intersection(current_idx, prev_movable, next_movable)
            )

            # if intersection_prev is not None:
            #     # If there is an intersection, we need to shift the neighbouring line
            #     prev_idx = self.get_prev_line_idx(current_idx)
            #     if prev_idx in shifted_lines:
            #         continue
            #     prev_line = get_line_local(prev_idx)
            #     new_prev_line = prev_line.through(intersection_prev)
            #     shifted_lines[prev_idx] = new_prev_line
            #     not_processed_lines.append(prev_idx)

            # if intersection_next is not None:
            #     # If there is an intersection, we need to shift the neighbouring line
            #     next_idx = self.get_next_line_idx(current_idx)
            #     if next_idx in shifted_lines:
            #         continue
            #     next_line = get_line_local(next_idx)
            #     new_next_line = next_line.through(intersection_next)
            #     shifted_lines[next_idx] = new_next_line
            #     not_processed_lines.append(next_idx)

            if prev_movable:
                prev_idx = self.get_prev_line_idx(current_idx)
                if intersection_prev is not None:
                    # If there is an intersection, we need to shift the neighbouring line
                    prev_line = get_line_local(prev_idx)
                    new_prev_line = prev_line.through(intersection_prev)
                    shifted_lines[prev_idx] = new_prev_line
                if continue_prev:
                    next_lines_to_process.append((prev_idx, True, False))

            if next_movable:
                next_idx = self.get_next_line_idx(current_idx)
                if intersection_next is not None:
                    # If there is an intersection, we need to shift the neighbouring line
                    next_line = get_line_local(next_idx)
                    new_next_line = next_line.through(intersection_next)
                    shifted_lines[next_idx] = new_next_line
                if continue_next:
                    next_lines_to_process.append((next_idx, False, True))

        return shifted_lines

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

    def _old_2_lines_to_move_if_line_is_shifted(
        self,
        idx: int,
        shift: Vector,
        callback: Callable[[int, Dict[int, Line]], None] = lambda i, s: None,
    ) -> Dict[int, Line]:
        shifted_lines = {idx: self.get_line(idx).shifted(shift)}

        def get_line_local(line_idx: int) -> Line:
            if line_idx in shifted_lines:
                return shifted_lines[line_idx]
            return self.get_line(line_idx)

        # Check for issues in the moved line
        line = get_line_local(idx)
        prev_idx = self.get_prev_line_idx(idx)
        prev_line = get_line_local(prev_idx)
        next_idx = self.get_next_line_idx(idx)
        next_line = get_line_local(next_idx)
        intersection_prev_point = prev_line.intersection_with_line(line).unwrap()
        intersection_next_point = line.intersection_with_line(next_line).unwrap()
        through_point = intersection_prev_point.on_line_to(intersection_next_point, 0.5)
        if is_reduced_to_point(line, prev_line, next_line):
            prev_through_point = through_point
            next_through_point = through_point
        else:
            if is_flipped(line, prev_line, next_line):
                prev_through_point = through_point
                next_through_point = through_point
                shifted_lines[prev_idx] = prev_line.through(prev_through_point)
                shifted_lines[next_idx] = next_line.through(next_through_point)
            else:
                prev_through_point = intersection_prev_point
                next_through_point = intersection_next_point

        # We want to process the prev and next lines in an interleaved manner,
        # always starting with the side that has the smallest angle with the main line
        prev_to_process_idx: Optional[int] = prev_idx
        next_to_process_idx: Optional[int] = next_idx
        prev_to_process_dot = get_line_local(prev_to_process_idx).dir_vector.dot(
            line.dir_vector
        )
        next_to_process_dot = get_line_local(next_to_process_idx).dir_vector.dot(
            line.dir_vector
        )
        num_indices = len(self.lines)
        last_index = idx
        processed_indices = set([idx])

        callback(idx, shifted_lines)

        while True:
            if prev_to_process_idx is None and next_to_process_idx is None:
                break

            # Bigger dot product means smaller angle
            do_next = prev_to_process_idx is None or (
                prev_to_process_dot < next_to_process_dot
            )

            if do_next:
                logging.debug(f"Processing next line index {next_to_process_idx}")
                assert next_to_process_idx is not None
                # Process the next line
                current_idx = next_to_process_idx
                current_prev_idx = self.get_prev_line_idx(current_idx)
                current_next_idx = self.get_next_line_idx(current_idx)
                current_line = get_line_local(current_idx)
                current_prev_line = get_line_local(current_prev_idx)
                current_next_line = get_line_local(current_next_idx)
                if not is_reduced_to_point(
                    current_line, current_prev_line, current_next_line
                ):
                    solved_issue = not is_flipped(
                        current_line, current_prev_line, current_next_line
                    )

                    if solved_issue:
                        logging.debug(
                            f"Line index {current_idx} is not flipped, no need to shift."
                        )

                    # Try to shift the current line through the intersection of the neighbours
                    if not solved_issue:
                        logging.debug(
                            f"Trying to shift line index {current_idx} through intersection of neighbours"
                        )
                        # TODO: Handle parallel case
                        next_prev_intersection = (
                            current_next_line.intersection_with_line(
                                current_prev_line
                            ).unwrap()
                        )
                        # Put the point on the segment
                        next_prev_intersection = (
                            current_prev_line.segment(
                                get_line_local(
                                    self.get_prev_line_idx(current_prev_idx)
                                ),
                                get_line_local(
                                    self.get_next_line_idx(current_prev_idx)
                                ),
                            )
                            .unwrap()
                            .projection_on_segment(next_prev_intersection)
                        )
                        new_current_line = current_line.through(next_prev_intersection)

                        reduced_to_point = is_reduced_to_point(
                            new_current_line, current_prev_line, current_next_line
                        )
                        flipped = is_flipped(
                            new_current_line, current_prev_line, current_next_line
                        )
                        if reduced_to_point or not flipped:
                            reason = (
                                "reduced to point"
                                if reduced_to_point
                                else "not flipped"
                            )
                            logging.debug(
                                f"Shifting line index {current_idx} through intersection of neighbours because it is {reason}"
                            )
                            logging.debug(
                                f"Current line: {current_line}, new current line: {new_current_line}, prev line: {current_prev_line}, next line: {current_next_line}"
                            )
                            shifted_lines[current_idx] = new_current_line
                            solved_issue = True
                            current_line = shifted_lines[current_idx]
                            next_through_point = next_prev_intersection

                    # # Try to shift the current line through the through point
                    # if not solved_issue:
                    #     new_current_line = current_line.through(next_through_point)
                    #     if not is_flipped(
                    #         new_current_line, current_prev_line, current_next_line
                    #     ):
                    #         shifted_lines[current_idx] = new_current_line
                    #         solved_issue = True
                    #         current_line = shifted_lines[current_idx]

                    # Try to shift the next line through the intersection of the previous and current lines
                    if not solved_issue:
                        logging.debug(
                            f"Trying to shift line index {current_next_idx} through intersection of previous and current line"
                        )
                        curr_prev_intersection = (
                            current_prev_line.intersection_with_line(
                                current_line
                            ).unwrap()
                        )
                        new_current_next_line = current_next_line.through(
                            curr_prev_intersection
                        )
                        if is_reduced_to_point(
                            current_line, current_prev_line, new_current_next_line
                        ) or not is_flipped(
                            current_line, current_prev_line, new_current_next_line
                        ):
                            logging.debug(
                                f"Shifting line index {current_next_idx} through intersection of previous and current line"
                            )
                            shifted_lines[current_next_idx] = new_current_next_line
                            solved_issue = True
                            current_next_line = shifted_lines[current_next_idx]
                            next_through_point = curr_prev_intersection

                    if not solved_issue:
                        logging.warning(
                            f"Could not resolve self-intersection for line index {current_idx}"
                        )
                else:
                    logging.debug(f"Line index {current_idx} is reduced to a point.")
                    next_through_point = current_line.intersection_with_line(
                        current_next_line
                    ).unwrap()

                processed_indices.add(current_idx)

                if current_next_idx in processed_indices:
                    next_to_process_idx = None
                    next_to_process_dot = -float("inf")
                else:
                    next_to_process_idx = current_next_idx
                    last_index = next_to_process_idx
                    next_to_process_dot = get_line_local(
                        next_to_process_idx
                    ).dir_vector.dot(line.dir_vector)

            else:
                logging.debug(f"Processing prev line index {prev_to_process_idx}")
                # Process the prev line
                assert prev_to_process_idx is not None
                current_idx = prev_to_process_idx
                current_prev_idx = self.get_prev_line_idx(current_idx)
                current_next_idx = self.get_next_line_idx(current_idx)
                current_line = get_line_local(current_idx)
                current_prev_line = get_line_local(current_prev_idx)
                current_next_line = get_line_local(current_next_idx)
                if not is_reduced_to_point(
                    current_line, current_prev_line, current_next_line
                ):
                    solved_issue = not is_flipped(
                        current_line, current_prev_line, current_next_line
                    )

                    if solved_issue:
                        logging.debug(
                            f"Line index {current_idx} is not flipped, no need to shift."
                        )

                    # Try to shift the current line through the intersection of the neighbours
                    if not solved_issue:
                        logging.debug(
                            f"Trying to shift line index {current_idx} through intersection of neighbours"
                        )
                        # TODO: Handle parallel case
                        next_prev_intersection = (
                            current_next_line.intersection_with_line(
                                current_prev_line
                            ).unwrap()
                        )
                        # Put the point on the segment
                        next_prev_intersection = (
                            current_next_line.segment(
                                get_line_local(
                                    self.get_prev_line_idx(current_next_idx)
                                ),
                                get_line_local(
                                    self.get_next_line_idx(current_next_idx)
                                ),
                            )
                            .unwrap()
                            .projection_on_segment(next_prev_intersection)
                        )
                        new_current_line = current_line.through(next_prev_intersection)

                        reduced_to_point = is_reduced_to_point(
                            new_current_line, current_prev_line, current_next_line
                        )
                        flipped = is_flipped(
                            new_current_line, current_prev_line, current_next_line
                        )
                        if reduced_to_point or not flipped:
                            reason = (
                                "reduced to point"
                                if reduced_to_point
                                else "not flipped"
                            )
                            logging.debug(
                                f"Shifting line index {current_idx} through intersection of neighbours because it is {reason}"
                            )
                            logging.debug(
                                f"Current line: {current_line}, new current line: {new_current_line}, prev line: {current_prev_line}, next line: {current_next_line}"
                            )
                            shifted_lines[current_idx] = new_current_line
                            solved_issue = True
                            current_line = shifted_lines[current_idx]
                            prev_through_point = next_prev_intersection

                    # # Try to shift the current line through the through point
                    # if not solved_issue:
                    #     new_current_line = current_line.through(prev_through_point)
                    #     if not is_flipped(
                    #         new_current_line, current_prev_line, current_next_line
                    #     ):
                    #         shifted_lines[current_idx] = new_current_line
                    #         solved_issue = True
                    #         current_line = shifted_lines[current_idx]

                    # Try to shift the previous line through the intersection of the next and current lines
                    if not solved_issue:
                        logging.debug(
                            f"Trying to shift line index {current_prev_idx} through intersection of next and current line"
                        )
                        curr_next_intersection = (
                            current_next_line.intersection_with_line(
                                current_line
                            ).unwrap()
                        )
                        new_current_prev_line = current_prev_line.through(
                            curr_next_intersection
                        )
                        if is_reduced_to_point(
                            current_line, new_current_prev_line, current_next_line
                        ) or not is_flipped(
                            current_line, new_current_prev_line, current_next_line
                        ):
                            logging.debug(
                                f"Shifting line index {current_prev_idx} through intersection of next and current line"
                            )
                            shifted_lines[current_prev_idx] = new_current_prev_line
                            solved_issue = True
                            current_prev_line = shifted_lines[current_prev_idx]
                            prev_through_point = curr_next_intersection

                    # if not solved_issue:
                    #     logging.warning(
                    #         f"Could not resolve self-intersection for line index {current_idx}"
                    #     )

                else:
                    logging.debug(f"Line index {current_idx} is reduced to a point.")
                    prev_through_point = current_line.intersection_with_line(
                        current_prev_line
                    ).unwrap()

                processed_indices.add(current_idx)

                if current_prev_idx in processed_indices:
                    prev_to_process_idx = None
                    prev_to_process_dot = -float("inf")
                else:
                    prev_to_process_idx = current_prev_idx
                    last_index = prev_to_process_idx
                    prev_to_process_dot = get_line_local(
                        prev_to_process_idx
                    ).dir_vector.dot(line.dir_vector)

            callback(current_idx, shifted_lines)
            logging.debug(f"Shifted lines so far: {list(shifted_lines.keys())}")

        return shifted_lines

    def _old_3_lines_to_move_if_line_is_shifted(
        self,
        idx: int,
        shift_dir: UnitVector,
        shift_length: float,
        callback: Callable[[int, Dict[int, Line]], None] = lambda i, s: None,
    ) -> Dict[int, Line]:
        if shift_length < 0:
            shift_dir = shift_dir.flipped()
            shift_length = -shift_length

        logging.info(
            f"Looking at line {idx} with shift dir {shift_dir} and shift length {shift_length:.5f}"
        )

        shifted_lines: Dict[int, Line] = {}
        shifts: Dict[int, float] = {}
        main_shift_length = shift_length

        def get_line_local(line_idx: int) -> Line:
            if line_idx in shifted_lines:
                return shifted_lines[line_idx]
            return self.get_line(line_idx)

        def set_shift(line_idx: int, _shift_length: float) -> None:
            logging.info(
                f"Shifting line index {line_idx} by {_shift_length:.5f} in direction {shift_dir}"
            )
            if line_idx not in shifts or _shift_length > shifts[line_idx]:
                _shift_length = main_shift_length
                shifted_lines[line_idx] = self.get_line(line_idx).shifted(
                    shift_dir * _shift_length
                )
                shifts[line_idx] = _shift_length

        set_shift(idx, main_shift_length)

        # For this version, all the lines that are moved will be moved by the same shift as the main line.
        # We will incrementally check for self-intersections and add new lines when there are self-intersections to fix.

        current_prev_idx = self.get_prev_line_idx(idx)
        current_prev_prev_idx = self.get_prev_line_idx(current_prev_idx)
        current_next_idx = self.get_next_line_idx(idx)
        current_next_next_idx = self.get_next_line_idx(current_next_idx)

        current_focus_line = get_line_local(idx)
        current_prev_line = get_line_local(current_prev_idx)
        current_prev_prev_line = get_line_local(current_prev_prev_idx)
        current_next_line = get_line_local(current_next_idx)
        current_next_next_line = get_line_local(current_next_next_idx)

        current_prev_line_flipped = is_problematic(
            current_prev_line, current_prev_prev_line, current_focus_line
        )
        current_focus_line_flipped = is_problematic(
            current_focus_line, current_prev_line, current_next_line
        )
        current_next_line_flipped = is_problematic(
            current_next_line, current_focus_line, current_next_next_line
        )

        if current_focus_line_flipped:
            logging.info(
                "Main line is flipped, trying to fix it by shifting the neighbours."
            )
            # Compute the distance in the shift direction between the intersection and the new line
            current_point = current_prev_line.intersection_with_line(
                current_next_line
            ).unwrap()
            shift_line = Line.from_point_and_dir(current_point, shift_dir)
            objective_point = shift_line.intersection_with_line(
                current_focus_line
            ).unwrap()
            current_shift_length = current_point.to(objective_point).dot(shift_dir)
            if current_shift_length < 0:
                logging.error(
                    f"Negative shift length ({current_shift_length:.5f}) for the neighbours of the main line in case of flipped main line should not happen."
                )
            else:
                if (
                    current_shift_length
                    > main_shift_length + EDGE_COMPUTATIONS_ERROR_TOLERANCE
                ):
                    logging.warning(
                        f"Shift length ({current_shift_length:.5f}) for the neighbours of the main line in case of flipped main line is bigger than the main shift length ({main_shift_length:.5f}), this should not happen."
                    )
                    current_shift_length = main_shift_length
                set_shift(current_prev_idx, current_shift_length)
                set_shift(current_next_idx, current_shift_length)
        if current_prev_line_flipped:
            logging.info("Previous line is flipped, trying to fix it by shifting it.")
            # Compute the distance in the shift direction between the intersection (prev_prev, main) and the prev
            objective_point = current_prev_prev_line.intersection_with_line(
                current_focus_line
            ).unwrap()
            shift_line = Line.from_point_and_dir(objective_point, shift_dir)
            current_point = shift_line.intersection_with_line(current_prev_line)
            # If there is no intersection, it means that the lines are parallel, so shifting doesn't matter
            if current_point.is_err():
                logging.info(
                    "Shift direction is parallel to the previous line in case of flipped previous line, not shifting as it would be counter-productive to the goal of keeping topology."
                )
            else:
                current_point = current_point.unwrap()
                current_shift_length = current_point.to(objective_point).dot(shift_dir)
                if current_shift_length < 0:
                    # Do nothing as it would be counter-productive to the goal of keeping topology
                    logging.info(
                        f"Negative shift length ({current_shift_length:.5f}) for the previous line in case of flipped previous line, not shifting as it would be counter-productive to the goal of keeping topology."
                    )
                else:
                    if (
                        current_shift_length
                        > main_shift_length + EDGE_COMPUTATIONS_ERROR_TOLERANCE
                    ):
                        # Do not shift more than the main shift length, as the rest of this issue should be resolved by shifting the prev_prev
                        current_shift_length = main_shift_length
                    set_shift(current_prev_idx, current_shift_length)
        if current_next_line_flipped:
            logging.info("Next line is flipped, trying to fix it by shifting it.")
            # Compute the distance in the shift direction between the intersection (next_next, main) and the next
            objective_point = current_next_next_line.intersection_with_line(
                current_focus_line
            ).unwrap()
            shift_line = Line.from_point_and_dir(objective_point, shift_dir)
            current_point = shift_line.intersection_with_line(current_next_line)
            if current_point.is_ok():
                current_point = current_point.unwrap()
                current_shift_length = current_point.to(objective_point).dot(shift_dir)
                if current_shift_length < 0:
                    # Do nothing as it would be counter-productive to the goal of keeping topology
                    logging.info(
                        "Negative shift length for the next line in case of flipped next line, not shifting as it would be counter-productive to the goal of keeping topology."
                    )
                else:
                    if (
                        current_shift_length
                        > main_shift_length + EDGE_COMPUTATIONS_ERROR_TOLERANCE
                    ):
                        # Do not shift more than the main shift length, as the rest of this issue should be resolved by shifting the next_next
                        current_shift_length = main_shift_length
                    set_shift(current_next_idx, current_shift_length)

        for _idx, shifted_line in shifted_lines.items():
            logging.debug(
                f"Shifted {_idx:>4}: {self.get_line(_idx)} to {shifted_line}\n"
            )
        callback(idx, shifted_lines)

        initial_focus_line_flipped = current_focus_line_flipped
        initial_prev_line_flipped = current_prev_line_flipped
        initial_next_line_flipped = current_next_line_flipped

        # if initial_focus_line_flipped or initial_prev_line_flipped:
        #     last_index = current_prev_idx
        # else:
        #     last_index = idx
        last_index = idx
        while True:
            current_focus_index = self.get_prev_line_idx(last_index)
            logging.info(
                f"Processing line index {current_focus_index} in backward direction."
            )
            current_next_idx = self.get_next_line_idx(current_focus_index)
            current_next_next_idx = self.get_next_line_idx(current_next_idx)
            current_prev_idx = self.get_prev_line_idx(current_focus_index)
            current_prev_prev_idx = self.get_prev_line_idx(current_prev_idx)

            current_focus_line = get_line_local(current_focus_index)
            current_next_line = get_line_local(current_next_idx)
            current_next_next_line = get_line_local(current_next_next_idx)
            current_prev_line = get_line_local(current_prev_idx)
            current_prev_prev_line = get_line_local(current_prev_prev_idx)

            current_next_line_flipped = is_problematic(
                current_next_line, current_focus_line, current_next_next_line
            )
            current_focus_line_flipped = is_problematic(
                current_focus_line, current_prev_line, current_next_line
            )
            current_prev_line_flipped = is_problematic(
                current_prev_line, current_prev_prev_line, current_focus_line
            )

            if (
                not current_focus_line_flipped
                and not current_prev_line_flipped
                and not current_next_line_flipped
            ):
                logging.info(f"Stopping at line index {current_focus_index}.")
                break

            # factors = [0.1 * i for i in range(1, 11)]
            # for factor in factors:
            #     if (
            #         current_focus_line_flipped
            #         or current_prev_line_flipped
            #         or current_next_line_flipped
            #     ):
            #         shifted_lines[current_focus_index] = self.get_line(
            #             current_focus_index
            #         ).shifted(shift * factor)
            #         current_line = shifted_lines[current_focus_index]

            #         current_next_line_flipped = is_flipped(
            #             current_next_line, current_line, current_next_next_line
            #         )
            #         current_line_flipped = is_flipped(
            #             current_line, current_prev_line, current_next_line
            #         )
            #         current_prev_line_flipped = is_flipped(
            #             current_prev_line, current_prev_prev_line, current_line
            #         )
            #     else:
            #         break

            # if current_next_line_flipped and current_focus_line_flipped:
            #     logging.error(
            #         "Both the current line and the next line are flipped, cannot fix the topology issue."
            #     )
            #     raise Exception(
            #         "Both the current line and the next line are flipped, cannot fix the topology issue."
            #     )

            final_shift_length = 0.0

            if current_next_line_flipped:
                logging.info(
                    "Next line is flipped, trying to fix it by shifting the current line."
                )
                # Compute the distance in the shift direction between the intersection and the new line
                objective_point = current_next_line.intersection_with_line(
                    current_next_next_line
                ).unwrap()
                shift_line = Line.from_point_and_dir(objective_point, shift_dir)
                current_point = shift_line.intersection_with_line(current_next_line)
                if current_point.is_err():
                    logging.error(
                        "Shift direction is parallel to the next line, cannot finish fixing the topology issue."
                    )
                else:
                    current_point = current_point.unwrap()
                    current_shift_length = current_point.to(objective_point).dot(
                        shift_dir
                    )
                    if current_shift_length < -1e-6:
                        logging.error(
                            f"Shift length is negative ({current_shift_length:.5f}), cannot finish fixing the topology issue."
                        )
                    else:
                        if current_shift_length > main_shift_length:
                            logging.error(
                                f"Shift length is too large ({current_shift_length:.5f} > {main_shift_length:.5f}), cannot finish fixing the topology issue."
                            )
                            current_shift_length = main_shift_length
                        elif current_shift_length < EDGE_DISPLACEMENTS_MIN_SHIFT:
                            logging.info(
                                f"Shift length ({current_shift_length:.5f}) is smaller than the minimum shift ({EDGE_DISPLACEMENTS_MIN_SHIFT:.5f}), will use the minimum shift to try to fix the topology issue."
                            )
                            current_shift_length = EDGE_DISPLACEMENTS_MIN_SHIFT
                        set_shift(current_focus_index, current_shift_length)

            if current_focus_line_flipped:
                logging.info(
                    "Current line is flipped, trying to fix it by shifting it."
                )
                # Compute the distance in the shift direction between the intersection and the new line
                objective_point = current_prev_line.intersection_with_line(
                    current_next_line
                ).unwrap()
                shift_line = Line.from_point_and_dir(objective_point, shift_dir)
                current_point = shift_line.intersection_with_line(current_focus_line)
                if current_point.is_err():
                    logging.warning(
                        "Shift direction is parallel to the current line, cannot start fixing the topology issue."
                    )
                else:
                    current_point = current_point.unwrap()
                    current_shift_length = current_point.to(objective_point).dot(
                        shift_dir
                    )
                    if current_shift_length < -1e-6:
                        logging.info(
                            "Negative shift length, will not move as it would be counter-productive to fixing the topology issue."
                        )
                    else:
                        if current_shift_length > main_shift_length:
                            logging.info(
                                f"Cannot fully fix the topology issue (shift length: {current_shift_length:.5f}, main shift length: {main_shift_length:.5f}), will be handled by the next iteration."
                            )
                            current_shift_length = main_shift_length
                        elif current_shift_length < EDGE_DISPLACEMENTS_MIN_SHIFT:
                            logging.info(
                                f"Shift length ({current_shift_length:.5f}) is smaller than the minimum shift ({EDGE_DISPLACEMENTS_MIN_SHIFT:.5f}), will use the minimum shift to try to fix the topology issue."
                            )
                            current_shift_length = EDGE_DISPLACEMENTS_MIN_SHIFT
                        set_shift(current_focus_index, current_shift_length)

            if current_prev_line_flipped:
                logging.warning(
                    "Flipped prev line in prev direction, should not happen."
                )

            last_index = current_focus_index
            callback(current_focus_index, shifted_lines)

        # if initial_focus_line_flipped or initial_next_line_flipped:
        #     last_index = current_prev_idx
        # else:
        #     last_index = idx
        last_index = idx
        while True:
            current_focus_index = self.get_next_line_idx(last_index)
            logging.info(
                f"Processing line index {current_focus_index} in forward direction."
            )
            current_prev_idx = self.get_prev_line_idx(current_focus_index)
            current_prev_prev_idx = self.get_prev_line_idx(current_prev_idx)
            current_next_idx = self.get_next_line_idx(current_focus_index)
            current_next_next_idx = self.get_next_line_idx(current_next_idx)

            current_prev_line = get_line_local(current_prev_idx)
            current_prev_prev_line = get_line_local(current_prev_prev_idx)
            current_focus_line = get_line_local(current_focus_index)
            current_next_line = get_line_local(current_next_idx)
            current_next_next_line = get_line_local(current_next_next_idx)

            current_prev_line_flipped = is_problematic(
                current_prev_line, current_prev_prev_line, current_focus_line
            )
            current_focus_line_flipped = is_problematic(
                current_focus_line, current_prev_line, current_next_line
            )
            current_next_line_flipped = is_problematic(
                current_next_line, current_focus_line, current_next_next_line
            )
            # if (
            #     current_line_flipped
            #     or current_prev_line_flipped
            #     or current_next_line_flipped
            # ):
            #     shifted_lines[current_index] = self.get_line(current_index).shifted(
            #         shift
            #     )
            #     current_line = shifted_lines[current_index]
            #     last_index = current_index
            #     callback(current_index, shifted_lines)
            # else:
            #     logging.debug(f"Stopping at line index {current_index}.")
            #     break

            if (
                not current_focus_line_flipped
                and not current_prev_line_flipped
                and not current_next_line_flipped
            ):
                logging.info(f"Stopping at line index {current_focus_index}.")
                break

            # factors = [0.1 * i for i in range(1, 11)]
            # for factor in factors:
            #     if (
            #         current_focus_line_flipped
            #         or current_prev_line_flipped
            #         or current_next_line_flipped
            #     ):
            #         shifted_lines[current_focus_index] = self.get_line(
            #             current_focus_index
            #         ).shifted(shift * factor)
            #         current_focus_line = shifted_lines[current_focus_index]

            #         current_next_line_flipped = is_flipped(
            #             current_next_line, current_focus_line, current_next_next_line
            #         )
            #         current_focus_line_flipped = is_flipped(
            #             current_focus_line, current_prev_line, current_next_line
            #         )
            #         current_prev_line_flipped = is_flipped(
            #             current_prev_line, current_prev_prev_line, current_focus_line
            #         )
            #     else:
            #         break

            if current_prev_line_flipped:
                logging.info(
                    "Previous line is flipped, trying to fix it by shifting the current line."
                )
                # Compute the distance in the shift direction between the intersection and the new line
                objective_point = current_prev_line.intersection_with_line(
                    current_prev_prev_line
                ).unwrap()
                shift_line = Line.from_point_and_dir(objective_point, shift_dir)
                current_point = shift_line.intersection_with_line(current_prev_line)
                if current_point.is_err():
                    logging.error(
                        "Shift direction is parallel to the previous line, cannot finish fixing the topology issue."
                    )
                else:
                    current_point = current_point.unwrap()
                    current_shift_length = current_point.to(objective_point).dot(
                        shift_dir
                    )
                    if current_shift_length < -1e-6:
                        logging.error(
                            f"Shift length is negative ({current_shift_length:.5f}), cannot finish fixing the topology issue."
                        )
                    else:
                        if current_shift_length > main_shift_length:
                            logging.error(
                                f"Shift length is too large ({current_shift_length:.5f} > {main_shift_length:.5f}), cannot finish fixing the topology issue."
                            )
                            current_shift_length = main_shift_length
                        elif current_shift_length < EDGE_DISPLACEMENTS_MIN_SHIFT:
                            logging.info(
                                f"Shift length ({current_shift_length:.5f}) is smaller than the minimum shift ({EDGE_DISPLACEMENTS_MIN_SHIFT:.5f}), will use the minimum shift to try to fix the topology issue."
                            )
                            current_shift_length = EDGE_DISPLACEMENTS_MIN_SHIFT
                        set_shift(current_focus_index, current_shift_length)
            if current_focus_line_flipped:
                logging.info(
                    "Current line is flipped, trying to fix it by shifting it."
                )
                # Compute the distance in the shift direction between the intersection and the new line
                objective_point = current_next_line.intersection_with_line(
                    current_prev_line
                ).unwrap()
                shift_line = Line.from_point_and_dir(objective_point, shift_dir)
                current_point = shift_line.intersection_with_line(current_focus_line)
                if current_point.is_err():
                    logging.warning(
                        "Shift direction is parallel to the current line, cannot start fixing the topology issue."
                    )
                else:
                    current_point = current_point.unwrap()
                    current_shift_length = current_point.to(objective_point).dot(
                        shift_dir
                    )
                    if current_shift_length < -1e-6:
                        logging.info(
                            "Negative shift length, will not move as it would be counter-productive to fixing the topology issue."
                        )
                    else:
                        if current_shift_length > main_shift_length:
                            logging.info(
                                f"Cannot fully fix the topology issue (shift length: {current_shift_length:.5f}, main shift length: {main_shift_length:.5f}), will be handled by the next iteration."
                            )
                            current_shift_length = main_shift_length
                        elif current_shift_length < EDGE_DISPLACEMENTS_MIN_SHIFT:
                            logging.info(
                                f"Shift length ({current_shift_length:.5f}) is smaller than the minimum shift ({EDGE_DISPLACEMENTS_MIN_SHIFT:.5f}), will use the minimum shift to try to fix the topology issue."
                            )
                            current_shift_length = EDGE_DISPLACEMENTS_MIN_SHIFT
                        set_shift(current_focus_index, current_shift_length)

            if current_next_line_flipped:
                logging.warning(
                    "Flipped next line in next direction, should not happen."
                )

            last_index = current_focus_index
            callback(current_focus_index, shifted_lines)

        return shifted_lines

    def lines_to_move_if_line_is_shifted(
        self,
        idx: int,
        shift_dir: UnitVector,
        shift_length: float,
        callback: Callable[[int, Dict[int, Line]], None] = lambda i, s: None,
    ) -> Dict[int, Line]:

        # For this version, all the lines that are moved will be moved by the same shift as the main line.
        # We will incrementally check for self-intersections and add new lines when there are self-intersections to fix.

        if shift_length < 0:
            shift_dir = shift_dir.flipped()
            shift_length = -shift_length

        logging.info(
            f"Looking at line {idx} with shift dir {shift_dir} and shift length {shift_length:.5f}"
        )

        shifts: Dict[int, float] = {}
        main_shift_length = shift_length

        def get_line_local(line_idx: int) -> Line:
            if line_idx in shifts:
                return self.get_line(line_idx).shifted(shift_dir * shifts[line_idx])
            return self.get_line(line_idx)

        def get_shifted_lines() -> Dict[int, Line]:
            return {line_idx: get_line_local(line_idx) for line_idx in shifts.keys()}

        # def compute_buffer(focus_line_idx: int) -> Buffer:
        #     prev_idx = self.get_prev_line_idx(focus_line_idx)
        #     next_idx = self.get_next_line_idx(focus_line_idx)
        #     prev_prev_idx = self.get_prev_line_idx(prev_idx)
        #     next_next_idx = self.get_next_line_idx(next_idx)

        #     focus_line = self.get_line(focus_line_idx)
        #     prev_line = self.get_line(prev_idx)
        #     next_line = self.get_line(next_idx)
        #     prev_prev_line = self.get_line(prev_prev_idx)
        #     next_next_line = self.get_line(next_next_idx)

        #     buffer = compute_buffer_self_moving(
        #         focus_line=focus_line,
        #         prev_line=prev_line,
        #         next_line=next_line,
        #         prev_prev_line=prev_prev_line,
        #         next_next_line=next_next_line,
        #         movement_dir=shift_dir,
        #     )
        #     logging.info(f"Buffer for line index {focus_line_idx}: {buffer}")
        #     return buffer

        def set_shift(line_idx: int, _shift_length: float) -> None:
            logging.info(
                f"Shifting line index {line_idx} by {_shift_length:.5f} in direction {shift_dir}"
            )
            shifts[line_idx] = _shift_length

        def add_shift(line_idx: int, _shift_length: float) -> None:
            logging.info(
                f"Adding shift of {_shift_length:.5f} to line index {line_idx} in direction {shift_dir}"
            )
            if line_idx in shifts:
                shifts[line_idx] += _shift_length
            else:
                shifts[line_idx] = _shift_length

        def get_new_shift_length(buffer: float, previous_shift_length: float) -> float:
            if buffer == float("inf"):
                return 0.0
            buffer_to_use = buffer * EDGE_DISPLACEMENTS_ALPHA_BUFFER
            if buffer_to_use > previous_shift_length:
                return 0.0
            return previous_shift_length - buffer_to_use

        def buffer_prev(
            focus_line_idx: int, dominating_line_idx: int, next_dominating_line_idx: int
        ) -> Tuple[float, int]:
            prev_line_idx = focus_line_idx
            prev_prev_line_idx = self.get_prev_line_idx(prev_line_idx)

            prev_line = get_line_local(prev_line_idx)
            prev_prev_line = get_line_local(prev_prev_line_idx)
            dominating_line = get_line_local(dominating_line_idx)
            next_dominating_line = get_line_local(next_dominating_line_idx)

            buffer, dominating_line_changed = compute_buffer_for_prev(
                dominating_line=dominating_line,
                next_dominating_line=next_dominating_line,
                focus_line=prev_line,
                prev_line=prev_prev_line,
                movement_dir=shift_dir,
            )

            if dominating_line_changed:
                new_dominating_line_idx = prev_line_idx
            else:
                new_dominating_line_idx = dominating_line_idx

            return buffer, new_dominating_line_idx

        def buffer_next(
            focus_line_idx: int, dominating_line_idx: int, prev_dominating_line_idx: int
        ) -> Tuple[float, int]:
            next_line_idx = focus_line_idx
            next_next_line_idx = self.get_next_line_idx(next_line_idx)

            next_line = get_line_local(next_line_idx)
            next_next_line = get_line_local(next_next_line_idx)
            dominating_line = get_line_local(dominating_line_idx)
            prev_dominating_line = get_line_local(prev_dominating_line_idx)

            buffer, dominating_line_changed = compute_buffer_for_next(
                dominating_line=dominating_line,
                prev_dominating_line=prev_dominating_line,
                focus_line=next_line,
                next_line=next_next_line,
                movement_dir=shift_dir,
            )

            if dominating_line_changed:
                new_dominating_line_idx = next_line_idx
            else:
                new_dominating_line_idx = dominating_line_idx

            return buffer, new_dominating_line_idx

        # TODO: Handle the first case better

        next_dominating_line_idx = idx
        next_prev_dominating_line_idx = self.get_prev_line_idx(idx)
        prev_dominating_line_idx = idx
        prev_next_dominating_line_idx = self.get_next_line_idx(idx)

        prev_buffered_until_now = 0.0
        next_buffered_until_now = 0.0

        for _idx, shift in shifts.items():
            shifted_line = get_line_local(_idx)
            logging.debug(
                f"Shifted {_idx:>4}: {self.get_line(_idx)} to {shifted_line}\n"
            )
        callback(idx, get_shifted_lines())

        prev_last_index = idx
        next_last_index = idx
        prev_finished = False
        next_finished = False
        current_shift = 0.0
        add_shift(idx, 0.0)
        while current_shift < main_shift_length:
            if prev_finished:
                look_prev = False
            elif next_finished:
                look_prev = True
            else:
                prev_focus_index = self.get_prev_line_idx(prev_last_index)
                prev_focus_buffer, prev_new_dominating_line_idx = buffer_prev(
                    prev_focus_index,
                    prev_dominating_line_idx,
                    prev_next_dominating_line_idx,
                )

                next_focus_index = self.get_next_line_idx(next_last_index)
                next_focus_buffer, next_new_dominating_line_idx = buffer_next(
                    next_focus_index,
                    next_dominating_line_idx,
                    next_prev_dominating_line_idx,
                )

                logging.info(
                    f"Got buffer {prev_focus_buffer:.5f} for previous line index {prev_focus_index} and buffer {next_focus_buffer:.5f} for next line index {next_focus_index}."
                )

                if (
                    prev_buffered_until_now + prev_focus_buffer
                    < next_buffered_until_now + next_focus_buffer
                ):
                    look_prev = True
                else:
                    look_prev = False

            if look_prev:
                prev_focus_index = self.get_prev_line_idx(prev_last_index)
                prev_focus_buffer, prev_new_dominating_line_idx = buffer_prev(
                    prev_focus_index,
                    prev_dominating_line_idx,
                    prev_next_dominating_line_idx,
                )
                logging.info(
                    f"Looking at previous line index {prev_focus_index} with buffer {prev_focus_buffer:.5f}."
                )

                # # Look at the prev side
                # prev_buffered_until_now += prev_focus_buffer

                # if prev_buffered_until_now > main_shift_length:
                #     logging.info(
                #         f"Previous shift length for line index {prev_focus_index} has been reduced to 0, stopping the backward iterations."
                #     )
                #     prev_finished = True
                #     continue

                for line_idx in shifts.keys():
                    add_shift(line_idx, prev_focus_buffer)
                add_shift(prev_focus_index, 0.0)
                current_shift += prev_focus_buffer

                prev_dominating_line_idx = prev_new_dominating_line_idx
                prev_last_index = prev_focus_index
                callback(prev_focus_index, get_shifted_lines())

            else:
                next_focus_index = self.get_next_line_idx(next_last_index)
                next_focus_buffer, next_new_dominating_line_idx = buffer_next(
                    next_focus_index,
                    next_dominating_line_idx,
                    next_prev_dominating_line_idx,
                )
                logging.info(
                    f"Looking at next line index {next_focus_index} with buffer {next_focus_buffer:.5f}."
                )

                # # Look at the next side
                # next_buffered_until_now += next_focus_buffer

                # if next_buffered_until_now > main_shift_length:
                #     logging.info(
                #         f"Next shift length for line index {next_focus_index} has been reduced to 0, stopping the forward iterations."
                #     )
                #     next_finished = True
                #     continue

                for line_idx in shifts.keys():
                    add_shift(line_idx, next_focus_buffer)
                add_shift(next_focus_index, 0.0)
                current_shift += next_focus_buffer

                next_dominating_line_idx = next_new_dominating_line_idx
                next_last_index = next_focus_index
                callback(next_focus_index, get_shifted_lines())

        return get_shifted_lines()
        return get_shifted_lines()
