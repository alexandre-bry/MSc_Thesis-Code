import math
import random
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path
from typing import (
    Callable,
    Dict,
    Generic,
    List,
    NoReturn,
    Optional,
    Sequence,
    Tuple,
    TypeVar,
    Union,
    overload,
)

import matplotlib.pyplot as plt
import tqdm
from matplotlib.axes import Axes
from matplotlib.patches import ArrowStyle

POINT_SIZE = 3
POINT_COLOR = "blue"

LINE_WIDTH = 2
LINE_COLOR = "green"

SEGMENT_WIDTH = LINE_WIDTH
SEGMENT_COLOR = "red"
SEGMENT_POINT_SIZE = 5
SEGMENT_POINT_COLOR = "orange"

POLYGON_WIDTH = LINE_WIDTH
POLYGON_COLOR = "purple"
POLYGON_POINT_SIZE = SEGMENT_POINT_SIZE
POLYGON_POINT_COLOR = SEGMENT_POINT_COLOR

EDGE_MATCHING_OFFSET_ABSOLUTE_MAX = 2.0
EDGE_MATCHING_OFFSET_STEP = 0.1

LINEAR_CRITERION_MAX_DISTANCE = 0.3

T = TypeVar("T")
E = TypeVar("E", bound=BaseException)
U = TypeVar("U")


@dataclass(frozen=True)
class Ok(Generic[T]):
    value: T


@dataclass(frozen=True)
class Err(Generic[E]):
    error: E


class Result(Generic[T, E]):
    """
    Rust-like Result type:
      - Ok(value)
      - Err(error)

    Use `unwrap()` / `expect()` to fail fast.
    """

    __slots__ = ("_inner",)

    def __init__(self, inner: Union[Ok[T], Err[E]]):
        self._inner = inner

    @staticmethod
    def ok(value: T) -> "Result[T, E]":
        return Result(Ok(value))

    @staticmethod
    def err(error: E) -> "Result[T, E]":
        return Result(Err(error))

    def is_ok(self) -> bool:
        return isinstance(self._inner, Ok)

    def is_err(self) -> bool:
        return isinstance(self._inner, Err)

    def unwrap(self) -> T:
        """Return value or raise stored error."""
        if isinstance(self._inner, Ok):
            return self._inner.value
        raise self._inner.error

    def expect(self, message: str) -> T:
        """Return value or raise RuntimeError(message) chained from stored error."""
        if isinstance(self._inner, Ok):
            return self._inner.value
        raise RuntimeError(message) from self._inner.error

    def unwrap_err(self) -> E:
        """Return error or raise if this is Ok."""
        if isinstance(self._inner, Err):
            return self._inner.error
        raise RuntimeError(f"Called unwrap_err on Ok({self._inner.value!r})")

    def map(self, fn: Callable[[T], U]) -> "Result[U, E]":
        if isinstance(self._inner, Ok):
            return Result.ok(fn(self._inner.value))
        return Result.err(self._inner.error)

    def map_err(self, fn: Callable[[E], BaseException]) -> "Result[T, BaseException]":
        if isinstance(self._inner, Err):
            return Result.err(fn(self._inner.error))
        return Result.ok(self._inner.value)

    def value_or(self, default: T) -> T:
        return self._inner.value if isinstance(self._inner, Ok) else default

    def __repr__(self) -> str:
        if isinstance(self._inner, Ok):
            return f"Result.Ok({self._inner.value!r})"
        return f"Result.Err({self._inner.error!r})"


class Geometry(ABC):

    @abstractmethod
    def plot(self, ax: Optional[Axes] = None):
        raise NotImplementedError("Subclasses must implement this method")

    @abstractmethod
    def __str__(self) -> str:
        raise NotImplementedError("Subclasses must implement this method")

    def __repr__(self) -> str:
        return self.__str__()


class Point(Geometry):
    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y

    def __str__(self) -> str:
        return f"Point({self.x:.10f}, {self.y:.10f})"

    def plot(
        self,
        ax: Optional[Axes] = None,
        color: str = POINT_COLOR,
        size: int = POINT_SIZE,
    ):
        if ax is None:
            ax = plt.gca()
        ax.plot(self.x, self.y, "o", markersize=size, color=color)

    def __add__(self, other: "Vector") -> "Point":
        return Point(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Vector") -> "Point":
        return Point(self.x - other.x, self.y - other.y)

    def to(self, other: "Point") -> "Vector":
        return Vector(other.x - self.x, other.y - self.y)

    def distance_to(self, other: "Point") -> float:
        return self.to(other).length()


class Vector(Geometry):
    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y

    def __str__(self) -> str:
        return f"Vector({self.x:.10f}, {self.y:.10f})"

    def plot(
        self,
        ax: Optional[Axes] = None,
        base_point: Optional[Point] = None,
        color: str = LINE_COLOR,
        width: int = LINE_WIDTH,
    ):
        if base_point is None:
            raise ValueError("Base point must be provided to plot a vector")
        xs = [base_point.x, base_point.x + self.x]
        ys = [base_point.y, base_point.y + self.y]
        if ax is None:
            ax = plt.gca()
        ax.annotate(
            "",
            xy=(xs[1], ys[1]),
            xytext=(xs[0], ys[0]),
            arrowprops=dict(
                arrowstyle=ArrowStyle("-|>", head_length=1, head_width=1),
                color=color,
                linewidth=width,
            ),
        )

    def __add__(self, other: "Vector") -> "Vector":
        return Vector(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Vector") -> "Vector":
        return Vector(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> "Vector":
        return Vector(self.x * scalar, self.y * scalar)

    def __rmul__(self, scalar: float) -> "Vector":
        return self.__mul__(scalar)

    def dot(self, other: "Vector") -> float:
        return self.x * other.x + self.y * other.y

    def length(self) -> float:
        return (self.x**2 + self.y**2) ** 0.5

    def normalized(self) -> "NormalizedVector":
        return NormalizedVector(self.x, self.y)


class NormalizedVector(Vector):
    x: float
    y: float

    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y
        self.normalize()

    def __str__(self) -> str:
        return f"NormalizedVector({self.x:.2f}, {self.y:.2f})"

    def normalize(self) -> None:
        length: float = (self.x**2 + self.y**2) ** 0.5
        if length > 0:
            self.x /= length
            self.y /= length

    def __mul__(self, scalar: float) -> Vector:
        return super().__mul__(scalar)


class Segment:
    def __init__(self, start: Point, end: Point) -> None:
        self.start = start
        self.end = end

    def __str__(self) -> str:
        return f"Segment({self.start}, {self.end})"

    @property
    def min_x(self) -> float:
        return min(self.start.x, self.end.x)

    @property
    def max_x(self) -> float:
        return max(self.start.x, self.end.x)

    @property
    def min_y(self) -> float:
        return min(self.start.y, self.end.y)

    @property
    def max_y(self) -> float:
        return max(self.start.y, self.end.y)

    def get_line(self) -> Line:
        return Line.from_points(self.start, self.end)

    def length(self) -> float:
        return (
            (self.end.x - self.start.x) ** 2 + (self.end.y - self.start.y) ** 2
        ) ** 0.5

    def is_within_bounding_box(self, point: Point) -> bool:
        return (
            self.min_x <= point.x <= self.max_x and self.min_y <= point.y <= self.max_y
        )

    def projection_of_point(self, point: Point) -> tuple[Point, bool]:
        """Projects a point onto the line defined by the segment and checks if the projection lies on the segment.

        Args:
            point (Point): The point to be projected.

        Returns:
            tuple[Point, bool]: The projected point and a boolean indicating if it lies on the segment.
        """
        line = Line.from_points(self.start, self.end)
        point_on_line = line.projection_of_point(point)
        if self.min_x > point_on_line.x:
            return self.start, False
        elif self.max_x < point_on_line.x:
            return self.end, False
        elif self.min_y > point_on_line.y:
            return self.start, False
        elif self.max_y < point_on_line.y:
            return self.end, False
        else:
            return point_on_line, True

    def distance_to_point(self, point: Point) -> Tuple[float, float]:
        """Computes the distance from a point to the segment.
        The distance is divided between the distance in the direction of the normal vector and the distance in the direction of the segment.
        The distance in the direction of the segment is defined as the distance between the projection on the line and the projection on the segment.

        Args:
            point (Point): The point from which the distance is calculated.

        Returns:
            [float, float]: The distance in the direction of the normal vector and the distance in the direction of the segment.
        """
        line = self.get_line()
        point_on_line = line.projection_of_point(point)
        normal_distance = point.distance_to(point_on_line)

        if self.min_x > point_on_line.x:
            segment_distance = point_on_line.distance_to(self.start)
        elif self.max_x < point_on_line.x:
            segment_distance = point_on_line.distance_to(self.end)
        elif self.min_y > point_on_line.y:
            segment_distance = point_on_line.distance_to(self.start)
        elif self.max_y < point_on_line.y:
            segment_distance = point_on_line.distance_to(self.end)
        else:
            segment_distance = 0.0

        return normal_distance, segment_distance

    def intersection(self, other: "Segment") -> Result[Point, Exception]:
        line1 = self.get_line()
        line2 = other.get_line()
        intersection_point = line1.intersection_with_line(line2)
        if intersection_point.is_err():
            return Result.err(intersection_point.unwrap_err())
        intersection_point = intersection_point.unwrap()
        if self.is_within_bounding_box(
            intersection_point
        ) and other.is_within_bounding_box(intersection_point):
            return Result.ok(intersection_point)
        return Result.err(ValueError("Segments do not intersect"))

    def plot(
        self,
        ax: Optional[Axes] = None,
        color: str = SEGMENT_COLOR,
        width: int = SEGMENT_WIDTH,
        point_color: Optional[str] = None,
        point_size: Optional[int] = None,
    ):
        xs = [self.start.x, self.end.x]
        ys = [self.start.y, self.end.y]
        if ax is None:
            ax = plt.gca()
        ax.plot(
            xs,
            ys,
            linewidth=width,
            color=color,
            marker="o" if point_color and point_size else None,
            markersize=point_size if point_color and point_size else None,
            markerfacecolor=point_color if point_color and point_size else None,
        )


class Line:
    def __init__(self, dir_vector: NormalizedVector, value: float) -> None:
        self.dir_vector = dir_vector
        self.normal_vector = NormalizedVector(-dir_vector.y, dir_vector.x)
        self.value = value

    def __str__(self) -> str:
        return f"Line(dir_vector={self.dir_vector}, value={self.value:.2f})"

    @property
    def base_point(self) -> Point:
        # Get a point on the line (the point where the line intersects the normal vector)
        return Point(
            self.normal_vector.x * self.value, self.normal_vector.y * self.value
        )

    @property
    def slope(self) -> float:
        if self.dir_vector.x == 0:
            return float("inf")  # Vertical line
        return self.dir_vector.y / self.dir_vector.x

    @classmethod
    def from_points(cls, point1: Point, point2: Point) -> "Line":
        # Create a line from two points
        dir_vector = point1.to(point2).normalized()
        normal_vector = NormalizedVector(-dir_vector.y, dir_vector.x)
        value = normal_vector.x * point1.x + normal_vector.y * point1.y
        return cls(dir_vector=dir_vector, value=value)

    def through(self, point: Point) -> "Line":
        # Set the line to pass through the given point
        value = self.normal_vector.x * point.x + self.normal_vector.y * point.y
        return Line(dir_vector=self.dir_vector, value=value)

    def projection_of_point(self, point: Point) -> Point:
        # Project a point onto the line
        distance = (
            self.normal_vector.x * point.x + self.normal_vector.y * point.y - self.value
        )
        projected_x = point.x - distance * self.normal_vector.x
        projected_y = point.y - distance * self.normal_vector.y
        return Point(projected_x, projected_y)

    def shifted(self, shift_vector: Vector) -> "Line":
        # Shift the line by the given vector
        new_value = (
            self.value
            + shift_vector.x * self.normal_vector.x
            + shift_vector.y * self.normal_vector.y
        )
        return Line(dir_vector=self.dir_vector, value=new_value)

    def intersection_with_line(self, other: "Line") -> Result[Point, Exception]:
        # Calculate the intersection point of this line with another line
        a1, b1 = self.normal_vector.x, self.normal_vector.y
        c1 = self.value
        a2, b2 = other.normal_vector.x, other.normal_vector.y
        c2 = other.value

        determinant = a1 * b2 - a2 * b1
        if abs(determinant) < 1e-10:
            return Result.err(ValueError("Lines are parallel, no intersection"))

        x = (b2 * c1 - b1 * c2) / determinant
        y = (a1 * c2 - a2 * c1) / determinant
        return Result.ok(Point(x, y))

    def segment(self, line_1: "Line", line_2: "Line") -> Result[Segment, Exception]:
        point1 = self.intersection_with_line(line_1)
        point2 = self.intersection_with_line(line_2)

        if point1.is_err() or point2.is_err():
            return Result.err(
                ValueError("Lines do not intersect properly to form a segment")
            )

        return Result.ok(Segment(point1.unwrap(), point2.unwrap()))

    def plot(
        self,
        ax: Optional[Axes] = None,
        color: str = LINE_COLOR,
        width: int = LINE_WIDTH,
        arrow_position: Optional[Point] = None,
    ):
        if ax is None:
            ax = plt.gca()
        ax.axline(
            xy1=(self.base_point.x, self.base_point.y),
            slope=self.slope,
            color=color,
            linewidth=width,
        )
        if arrow_position:
            self.dir_vector.plot(
                ax=ax, base_point=arrow_position, color=color, width=width
            )


class Polygon(Geometry):
    def __init__(self, lines: list[Line]) -> None:
        self.lines = lines

    def __str__(self) -> str:
        return f"Polygon with {len(self.lines)} edges"

    @classmethod
    def from_points(cls, points: list[Point]) -> "Polygon":
        lines = []
        for i in range(len(points)):
            line = Line.from_points(points[i], points[(i + 1) % len(points)])
            lines.append(line)
        return cls(lines)

    def get_point(self, idx: int) -> Point:
        prev_idx = (idx - 1) % len(self.lines)
        point = self.lines[prev_idx].intersection_with_line(self.lines[idx])
        return point.unwrap()

    def get_points(self) -> list[Point]:
        return [self.get_point(i) for i in range(len(self.lines))]

    def get_segment(self, idx: int) -> Segment:
        return Segment(self.get_point(idx), self.get_point((idx + 1) % len(self.lines)))

    def get_segments(self) -> list[Segment]:
        return [self.get_segment(i) for i in range(len(self.lines))]

    def plot(
        self,
        ax: Optional[Axes] = None,
        color: str = POLYGON_COLOR,
        width: int = POLYGON_WIDTH,
        point_color: Optional[str] = POLYGON_POINT_COLOR,
        point_size: Optional[int] = POLYGON_POINT_SIZE,
    ):
        if ax is None:
            ax = plt.gca()
        for i in range(len(self.lines)):
            start_point = self.get_point(i)
            end_point = self.get_point((i + 1) % len(self.lines))
            Segment(start_point, end_point).plot(ax=ax, color=color, width=width)

        if point_color and point_size:
            for i in range(len(self.lines)):
                self.get_point(i).plot(ax=ax, color=point_color, size=point_size)


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
    # Maybe an idea would be to think about al possible cases
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
            intersection_point_projected = line.projection_of_point(intersection_point)
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
            tqdm.tqdm.write(
                "Previous neighbour is not movable, cannot resolve self-intersection"
            )
            tqdm.tqdm.write(f"{point_prev_prev.to(point_prev)=}")
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
            tqdm.tqdm.write(
                "Next neighbour is not movable, cannot resolve self-intersection"
            )
            tqdm.tqdm.write(f"{point_next.to(point_next_next)=}")
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
        return segment_result

    def get_segments(self) -> list[Segment]:
        return [self.get_segment(i).unwrap() for i in range(len(self.lines))]

    def lines_to_move_if_line_is_shifted(
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
            # tqdm.tqdm.write(f"Processing line index {not_processed_lines[0]}")
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

    # def plot(self, ax: Optional[Axes] = None, mode: str = "lines"):
    #     if mode == "lines":
    #         for line in self.lines:
    #             line.plot(ax=ax)
    #     elif mode == "segments":
    #         for i in range(len(self.lines)):
    #             segment_result = self.get_segment(i)
    #             segment_result.unwrap().plot(ax=ax)
    #     else:
    #         raise ValueError(f"Unknown plot mode: {mode}")


class Criterion(ABC):

    @abstractmethod
    def evaluate_segment(self, segment: Segment) -> float:
        raise NotImplementedError("Subclasses must implement this method")


class LinearCriterion(Criterion):

    def __init__(
        self, points: list[Point], weights: list[float], max_distance: float
    ) -> None:
        # Check that weights and points have the same length
        if len(points) != len(weights):
            raise ValueError("Points and weights must have the same length")

        self.points = points
        self.weights = weights
        self.max_distance = max_distance

    def evaluate_segment(self, segment: Segment) -> float:
        criterion_value = 0.0
        for point, weight in zip(self.points, self.weights):
            normal_distance, segment_distance = segment.distance_to_point(point)
            if normal_distance > self.max_distance:
                continue
            if segment_distance > 0:
                continue
            criterion_value += weight * (1.0 - normal_distance / self.max_distance)
        return criterion_value


class EdgeShiftingAlgorithm:
    def __init__(
        self,
        all_lines: AllLines,
        criterion: Criterion,
    ) -> None:
        self.all_lines = all_lines
        self.criterion = criterion

    def evaluate_lines(self, idxs: list[int], moved_lines: dict[int, Line]) -> float:
        total_criterion = 0.0

        def get_line_local(line_idx: int) -> Line:
            if line_idx in moved_lines:
                return moved_lines[line_idx]
            return self.all_lines.get_line(line_idx)

        for idx in idxs:
            prev_line = get_line_local(self.all_lines.get_prev_line_idx(idx))
            line = get_line_local(idx)
            next_line = get_line_local(self.all_lines.get_next_line_idx(idx))

            segment = line.segment(prev_line, next_line).unwrap()
            total_criterion += self.criterion.evaluate_segment(segment)

        return total_criterion

    def optimize_one_line(self, idx: int) -> Dict[int, Line]:
        line = self.all_lines.get_line(idx)
        shift_direction = line.normal_vector

        # Prepare all the offsets
        offsets = []
        max_offset_multiplier = int(
            EDGE_MATCHING_OFFSET_ABSOLUTE_MAX / EDGE_MATCHING_OFFSET_STEP
        )
        for offset_multiplier in range(
            -max_offset_multiplier, max_offset_multiplier + 1
        ):
            offsets.append(offset_multiplier * EDGE_MATCHING_OFFSET_STEP)

        # Find all the lines to move in the worst cases
        lines_moved_min = self.all_lines.lines_to_move_if_line_is_shifted(
            idx, shift_direction * offsets[0]
        )
        lines_moved_max = self.all_lines.lines_to_move_if_line_is_shifted(
            idx, shift_direction * offsets[-1]
        )
        lines_to_move = set(lines_moved_min.keys()) | set(lines_moved_max.keys())

        # Evaluate the criterion for each offset
        best_criterion_value = -float("inf")
        best_shifts: Dict[int, Line] = {}
        for offset in tqdm.tqdm(offsets, desc="Evaluating offsets", leave=False):
            shifted_lines = self.all_lines.lines_to_move_if_line_is_shifted(
                idx, shift_direction * offset
            )
            criterion_value = self.evaluate_lines(list(lines_to_move), shifted_lines)
            if criterion_value > best_criterion_value:
                best_criterion_value = criterion_value
                best_shifts = shifted_lines

        return best_shifts

    def optimize_all_lines(
        self,
        callback_after_optimization: Callable[[int], None] = lambda i: None,
    ) -> None:
        # Sort lines by length in descending order
        lengths = [
            self.all_lines.get_segment(i).unwrap().length()
            for i in range(len(self.all_lines.lines))
        ]
        sorted_indices = sorted(
            range(len(lengths)), key=lambda i: lengths[i], reverse=True
        )

        # Optimize lines in order of their lengths
        for idx in tqdm.tqdm(sorted_indices, desc="Optimizing lines"):
            best_shifts = self.optimize_one_line(idx)
            for line_idx, new_line in best_shifts.items():
                self.all_lines.update_line(line_idx, new_line)
            callback_after_optimization(idx)


@dataclass
class PlotStyle:
    point_size: int = POINT_SIZE
    point_color: str = POINT_COLOR
    line_width: int = LINE_WIDTH
    line_color: str = LINE_COLOR
    segment_width: int = SEGMENT_WIDTH
    segment_color: str = SEGMENT_COLOR
    segment_point_size: int = SEGMENT_POINT_SIZE
    segment_point_color: str = SEGMENT_POINT_COLOR
    polygon_width: int = POLYGON_WIDTH
    polygon_color: str = POLYGON_COLOR
    polygon_point_size: int = POLYGON_POINT_SIZE
    polygon_point_color: str = POLYGON_POINT_COLOR


@dataclass
class Snapshot:
    name: str
    points: list[Point] = field(default_factory=list)
    segments: list[Segment] = field(default_factory=list)
    polygons: list[Polygon] = field(default_factory=list)
    title: Optional[str] = None


class PlotRecorder:
    """
    Stores named snapshots of geometry and can render/show/save them.
    """

    def __init__(self, style: Optional[PlotStyle] = None) -> None:
        self.style = style or PlotStyle()
        self._snapshots: dict[str, Snapshot] = {}

    def capture(
        self,
        name: str,
        *,
        points: Optional[Sequence[Point]] = None,
        segments: Optional[Sequence[Segment]] = None,
        polygons: Optional[Sequence[Polygon]] = None,
        title: Optional[str] = None,
    ) -> None:
        self._snapshots[name] = Snapshot(
            name=name,
            points=list(points or []),
            segments=list(segments or []),
            polygons=list(polygons or []),
            title=title,
        )

    def snapshot_names(self) -> list[str]:
        return list(self._snapshots.keys())

    def render(self, name: str, show: bool = True) -> None:
        if name not in self._snapshots:
            raise ValueError(f"Unknown snapshot '{name}'")

        snap = self._snapshots[name]
        plt.figure()

        for p in snap.points:
            p.plot(color=self.style.point_color, size=self.style.point_size)

        for s in snap.segments:
            s.plot(
                color=self.style.segment_color,
                width=self.style.segment_width,
                point_color=self.style.segment_point_color,
                point_size=self.style.segment_point_size,
            )

        for poly in snap.polygons:
            poly.plot(
                color=self.style.polygon_color,
                width=self.style.polygon_width,
                point_color=self.style.polygon_point_color,
                point_size=self.style.polygon_point_size,
            )

        plt.title(snap.title or snap.name)
        plt.xlabel("X-axis")
        plt.ylabel("Y-axis")
        plt.axis("equal")
        plt.grid(True)

        if show:
            plt.show()

    def save(self, name: str, output_path: Path, dpi: int = 150) -> None:
        if name not in self._snapshots:
            raise ValueError(f"Unknown snapshot '{name}'")

        snap = self._snapshots[name]
        plt.figure()

        for p in snap.points:
            p.plot(color=self.style.point_color, size=self.style.point_size)

        for s in snap.segments:
            s.plot(color=self.style.segment_color, width=self.style.segment_width)

        for poly in snap.polygons:
            poly.plot(
                color=self.style.polygon_color,
                width=self.style.polygon_width,
                point_color=self.style.polygon_point_color,
                point_size=self.style.polygon_point_size,
            )

        plt.title(snap.title or snap.name)
        plt.xlabel("X-axis")
        plt.ylabel("Y-axis")
        plt.axis("equal")
        plt.grid(True)

        output_path.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(output_path, dpi=dpi, bbox_inches="tight")
        plt.close()

    def save_all(self, output_dir: Path, dpi: int = 150) -> None:
        output_dir.mkdir(parents=True, exist_ok=True)
        for name in self.snapshot_names():
            self.save(name, output_dir / f"{name}.png", dpi=dpi)

    def save_all_combined(self, output_path: Path, dpi: int = 150) -> None:
        """Save all snapshots as subplots in a single figure."""
        names = self.snapshot_names()
        if not names:
            return

        # Calculate grid dimensions
        num_snapshots = len(names)
        num_cols = int(math.ceil(math.sqrt(num_snapshots)))
        num_rows = int(math.ceil(num_snapshots / num_cols))

        # Create figure with subplots
        fig, axes = plt.subplots(
            num_rows, num_cols, figsize=(5 * num_cols, 5 * num_rows)
        )

        # Flatten axes array if only one row or column
        if num_rows == 1 and num_cols == 1:
            axes = [axes]
        elif num_rows == 1 or num_cols == 1:
            axes = axes.flat
        else:
            axes = axes.flat

        # Plot each snapshot
        for idx, name in enumerate(names):
            ax = axes[idx]
            snap = self._snapshots[name]

            # Plot on this axis
            for p in snap.points:
                p.plot(ax=ax, color=self.style.point_color, size=self.style.point_size)

            for s in snap.segments:
                s.plot(
                    ax=ax,
                    color=self.style.segment_color,
                    width=self.style.segment_width,
                    point_color=self.style.segment_point_color,
                    point_size=self.style.segment_point_size,
                )

            for poly in snap.polygons:
                poly.plot(
                    ax=ax,
                    color=self.style.polygon_color,
                    width=self.style.polygon_width,
                    point_color=self.style.polygon_point_color,
                    point_size=self.style.polygon_point_size,
                )

            ax.set_title(snap.title or snap.name)
            ax.set_xlabel("X-axis")
            ax.set_ylabel("Y-axis")
            ax.set_aspect("equal")
            ax.grid(True)

        # Hide unused subplots
        for idx in range(num_snapshots, len(axes)):
            axes[idx].set_visible(False)

        plt.tight_layout()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(output_path, dpi=dpi, bbox_inches="tight")
        plt.close()


def generate_points_circle(
    center: Point,
    radius: float,
    num_points: int,
    noise: float = 0.0,
) -> list[Point]:
    points = []
    for i in range(num_points):
        angle = (2 * math.pi * i) / num_points
        x = center.x + radius * math.cos(angle) + random.uniform(-noise, noise)
        y = center.y + radius * math.sin(angle) + random.uniform(-noise, noise)
        points.append(Point(x, y))
    return points


def generate_polygon_circle(
    center: Point, radius: float, num_vertices: int, noise: float = 0.0
) -> Polygon:
    points = []
    for i in range(num_vertices):
        angle = (2 * math.pi * i) / num_vertices
        x = center.x + radius * math.cos(angle) + random.uniform(-noise, noise)
        y = center.y + radius * math.sin(angle) + random.uniform(-noise, noise)
        points.append(Point(x, y))
    return Polygon.from_points(points)


def main():
    # Points
    center_points = Point(0, 0)
    radius_points = 10
    num_points = 100
    noise_points = 0.5

    # Polygon
    center_polygon = center_points + Vector(0, 1)
    radius_polygon = radius_points - 1
    num_vertices_polygon = 20
    noise_polygon = 0.5

    optimization_steps = 1

    random.seed(1)

    points = generate_points_circle(
        center=center_points,
        radius=radius_points,
        num_points=num_points,
        noise=noise_points,
    )
    polygon = generate_polygon_circle(
        center=center_polygon,
        radius=radius_polygon,
        num_vertices=num_vertices_polygon,
        noise=noise_polygon,
    )

    all_lines = AllLines(
        lines=polygon.lines,
        prev_lines=[(i - 1) % len(polygon.lines) for i in range(len(polygon.lines))],
        next_lines=[(i + 1) % len(polygon.lines) for i in range(len(polygon.lines))],
        touching_lines=[[] for _ in polygon.lines],
    )

    recorder_iterations = PlotRecorder()
    recorder_steps = PlotRecorder()

    recorder_iterations.capture(
        "initial",
        points=points,
        segments=all_lines.get_segments(),
        title="Initial state",
    )
    recorder_steps.capture(
        "initial",
        points=points,
        segments=all_lines.get_segments(),
        title="Initial state",
    )

    algo = EdgeShiftingAlgorithm(
        all_lines,
        LinearCriterion(
            points,
            [1.0] * len(points),
            max_distance=LINEAR_CRITERION_MAX_DISTANCE,
        ),
    )

    for step in tqdm.trange(1, optimization_steps + 1, desc="Optimization steps"):

        optimization_callback = lambda i: recorder_steps.capture(
            f"optimization_step_{step:{f'0{len(str(optimization_steps))}d'}}_line_{i}",
            points=points,
            segments=all_lines.get_segments(),
            title=f"After optimization step {step} (line {i})",
        )

        algo.optimize_all_lines(callback_after_optimization=optimization_callback)

        recorder_iterations.capture(
            f"optimization_step_{step:{f'0{len(str(optimization_steps))}d'}}",
            points=points,
            segments=all_lines.get_segments(),
            title=f"After optimization step {step}",
        )

    recorder_iterations.save_all_combined(Path("images/optimization_iterations.png"))
    recorder_steps.save_all_combined(Path("images/optimization_steps.png"))


if __name__ == "__main__":
    main()
