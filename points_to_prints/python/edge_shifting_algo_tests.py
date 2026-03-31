import copy
import logging
import math
import random
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path
from typing import (
    Annotated,
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
import typer
from custom_logging import LoggingContext, Verbose, setup_logging
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

EDGE_DISPLACEMENTS_MIN_SHIFT = 0.01
EDGE_DISPLACEMENTS_ALPHA_BUFFER = 1.0
EDGE_COMPUTATIONS_ERROR_TOLERANCE = 1e-6

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

    def on_line_to(self, other: "Point", t: float) -> "Point":
        return self + self.to(other) * t

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
        self._normalize()

    def __str__(self) -> str:
        return f"NormalizedVector({self.x:.2f}, {self.y:.2f})"

    def _normalize(self) -> None:
        length: float = (self.x**2 + self.y**2) ** 0.5
        if length > 0:
            self.x /= length
            self.y /= length

    def flipped(self) -> "NormalizedVector":
        return NormalizedVector(-self.x, -self.y)

    def __mul__(self, scalar: float) -> Vector:
        return super().__mul__(scalar)


class Segment:
    def __init__(self, start: Point, end: Point, name: Optional[str] = None) -> None:
        self.start = start
        self.end = end
        self.name = name

    def __str__(self) -> str:
        return f"Segment({self.start}, {self.end}, {self.name})"

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

    def projection_on_segment(self, point: Point) -> Point:
        point_on_line, is_on_segment = self.projection_on_line(point)
        if is_on_segment:
            return point_on_line
        else:
            dist_to_start = point_on_line.distance_to(self.start)
            dist_to_end = point_on_line.distance_to(self.end)
            return self.start if dist_to_start < dist_to_end else self.end

    def projection_on_line(self, point: Point) -> tuple[Point, bool]:
        """Projects a point onto the line defined by the segment and checks if the projection lies on the segment.

        Args:
            point (Point): The point to be projected.

        Returns:
            tuple[Point, bool]: The projected point and a boolean indicating if it lies on the segment.
        """
        line = Line.from_points(self.start, self.end)
        point_on_line = line.projection_on_line(point)
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
        point_on_line = line.projection_on_line(point)
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
        if self.name is not None:
            text_point = self.start.on_line_to(self.end, 0.5)
            ax.annotate(
                self.name,
                xy=(text_point.x, text_point.y),
                xytext=(text_point.x, text_point.y),
            )


class Line:
    def __init__(self, dir_vector: NormalizedVector, value: float) -> None:
        self.dir_vector = dir_vector
        self.normal_vector = NormalizedVector(-dir_vector.y, dir_vector.x)
        self.value = value

    def __str__(self) -> str:
        return f"Line(dir_vector={self.dir_vector}, base_point={self.base_point})"

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

    @classmethod
    def from_point_and_dir(cls, point: Point, dir_vector: Vector) -> "Line":
        # Create a line from a point and a direction vector
        dir_vector = dir_vector.normalized()
        normal_vector = NormalizedVector(-dir_vector.y, dir_vector.x)
        value = normal_vector.x * point.x + normal_vector.y * point.y
        return cls(dir_vector=dir_vector, value=value)

    def through(self, point: Point) -> "Line":
        # Set the line to pass through the given point
        value = self.normal_vector.x * point.x + self.normal_vector.y * point.y
        return Line(dir_vector=self.dir_vector, value=value)

    def projection_on_line(self, point: Point) -> Point:
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
            return Result.err(
                ValueError(f"Lines ({self}, {other}) are parallel, no intersection")
            )

        x = (b2 * c1 - b1 * c2) / determinant
        y = (a1 * c2 - a2 * c1) / determinant
        return Result.ok(Point(x, y))

    def segment(self, line_1: "Line", line_2: "Line") -> Result[Segment, Exception]:
        point1 = self.intersection_with_line(line_1)
        point2 = self.intersection_with_line(line_2)

        if point1.is_err():
            return Result.err(
                ValueError(
                    f"Lines ({self}, {line_1}) do not intersect properly to form a segment"
                )
            )
        if point2.is_err():
            return Result.err(
                ValueError(
                    f"Lines ({self}, {line_2}) do not intersect properly to form a segment"
                )
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
    movement_dir: NormalizedVector,
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
    movement_dir: NormalizedVector,
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


def compute_buffer_for_prev(
    next_dominating_line: Line,
    dominating_line: Line,
    prev_line: Line,
    prev_prev_line: Line,
    movement_dir: NormalizedVector,
) -> Tuple[float, bool]:
    # We consider that the focus line and the next line will move together along the movement direction
    limit = float("inf")
    dominating_line_changed = False

    # 1. Compute the limits with the previous line
    limit_point = prev_line.intersection_with_line(prev_prev_line).unwrap()
    limit_line = Line.from_point_and_dir(limit_point, movement_dir)
    origin_point = limit_line.intersection_with_line(dominating_line).unwrap()
    if origin_point.to(limit_point).dot(movement_dir) > 0:
        new_limit = origin_point.distance_to(limit_point)
        if new_limit < limit:
            limit = new_limit
            dominating_line_changed = False

    # 2. Compute the limits with the dominating line itself
    limit_point = prev_line.intersection_with_line(next_dominating_line).unwrap()
    limit_line = Line.from_point_and_dir(limit_point, movement_dir)
    origin_point = limit_line.intersection_with_line(dominating_line).unwrap()
    if origin_point.to(limit_point).dot(movement_dir) > 0:
        new_limit = origin_point.distance_to(limit_point)
        if new_limit < limit:
            limit = new_limit
            dominating_line_changed = True

    return limit, dominating_line_changed


def compute_buffer_for_next(
    prev_dominating_line: Line,
    dominating_line: Line,
    next_line: Line,
    next_next_line: Line,
    movement_dir: NormalizedVector,
) -> Tuple[float, bool]:
    # We consider that the focus line and the previous line will move together along the movement direction
    limit = float("inf")
    dominating_line_changed = False

    # 1. Compute the limits with the next line
    limit_point = next_line.intersection_with_line(next_next_line).unwrap()
    limit_line = Line.from_point_and_dir(limit_point, movement_dir)
    origin_point = limit_line.intersection_with_line(dominating_line).unwrap()
    if origin_point.to(limit_point).dot(movement_dir) > 0:
        new_limit = origin_point.distance_to(limit_point)
        if new_limit < limit:
            limit = new_limit
            dominating_line_changed = False

    # 2. Compute the limits with the focus line itself
    limit_point = next_line.intersection_with_line(prev_dominating_line).unwrap()
    limit_line = Line.from_point_and_dir(limit_point, movement_dir)
    origin_point = limit_line.intersection_with_line(dominating_line).unwrap()
    if origin_point.to(limit_point).dot(movement_dir) > 0:
        new_limit = origin_point.distance_to(limit_point)
        if new_limit < limit:
            limit = new_limit
            dominating_line_changed = True

    return limit, dominating_line_changed


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
        shift_dir: NormalizedVector,
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
        shift_dir: NormalizedVector,
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
                prev_line=prev_line,
                prev_prev_line=prev_prev_line,
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
                next_line=next_line,
                next_next_line=next_next_line,
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
            idx=idx, shift_dir=shift_direction, shift_length=offsets[0]
        )
        lines_moved_max = self.all_lines.lines_to_move_if_line_is_shifted(
            idx=idx, shift_dir=shift_direction, shift_length=offsets[-1]
        )
        lines_to_move = set(lines_moved_min.keys()) | set(lines_moved_max.keys())

        # Evaluate the criterion for each offset
        best_criterion_value = -float("inf")
        best_offset = 0.0
        best_shifts: Dict[int, Line] = {}
        for offset in tqdm.tqdm(offsets, desc="Evaluating offsets", leave=False):
            shifted_lines = self.all_lines.lines_to_move_if_line_is_shifted(
                idx=idx, shift_dir=shift_direction, shift_length=offset
            )
            criterion_value = self.evaluate_lines(list(lines_to_move), shifted_lines)
            if criterion_value > best_criterion_value:
                best_criterion_value = criterion_value
                best_shifts = shifted_lines
                best_offset = offset
        logging.info(f"Best offset {best_offset:.5f}")

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
        # sorted_indices = sorted(
        #     range(len(lengths)), key=lambda i: lengths[i], reverse=True
        # )
        sorted_indices = list(range(len(self.all_lines.lines)))
        random.shuffle(sorted_indices)

        # Optimize lines in order of their lengths
        for idx in tqdm.tqdm(sorted_indices, desc="Optimizing lines", leave=False):
            best_shifts = self.optimize_one_line(idx)
            for line_idx, new_line in best_shifts.items():
                self.all_lines.update_line(line_idx, new_line)
            any_flipped, flipped_index = self.all_lines.any_problem()
            if any_flipped:
                logging.warning(
                    f"Warning: Self-intersection detected at line index {flipped_index} after optimization of line index {idx}"
                )
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
    bounds: Optional[Tuple[Tuple[float, float], Tuple[float, float]]] = None


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
        bounds: Optional[Tuple[Tuple[float, float], Tuple[float, float]]] = None,
    ) -> None:
        self._snapshots[name] = Snapshot(
            name=name,
            points=list(points or []),
            segments=list(segments or []),
            polygons=list(polygons or []),
            title=title,
            bounds=bounds,
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
            ax: Axes = axes[idx]
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
            ax.set_xlim(snap.bounds[0][0], snap.bounds[1][0]) if snap.bounds else None
            ax.set_ylim(snap.bounds[0][1], snap.bounds[1][1]) if snap.bounds else None
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
        x = center.x + (radius + random.gauss(0, noise)) * math.cos(angle)
        y = center.y + (radius + random.gauss(0, noise)) * math.sin(angle)
        points.append(Point(x, y))
    return points


def generate_polygon_circle(
    center: Point, radius: float, num_vertices: int, noise: float = 0.0
) -> Polygon:
    points = []
    for i in range(num_vertices):
        angle = (2 * math.pi * i) / num_vertices
        x = center.x + (radius + random.gauss(0, noise)) * math.cos(angle)
        y = center.y + (radius + random.gauss(0, noise)) * math.sin(angle)
        points.append(Point(x, y))
    return Polygon.from_points(points)


def example_circle(
    num_points: int,
    num_vertices: int,
    noise_points: float,
    noise_polygon: float,
    radius_points: float,
    radius_polygon: float,
    shift_polygon_center: Vector,
    random_seed: Optional[int] = None,
) -> Tuple[list[Point], Polygon]:
    if random_seed is not None:
        random.seed(random_seed)

    center_points = Point(0, 0)
    center_polygon = center_points + shift_polygon_center

    points = generate_points_circle(
        center=center_points,
        radius=radius_points,
        num_points=num_points,
        noise=noise_points,
    )
    polygon = generate_polygon_circle(
        center=center_polygon,
        radius=radius_polygon,
        num_vertices=num_vertices,
        noise=noise_polygon,
    )

    return points, polygon


app = typer.Typer()


@app.command(
    "test_circle",
    help="Test the edge shifting algorithm on a circle example.",
)
def test_circle(
    optimization_iterations: Annotated[
        int,
        typer.Option(
            "--optimization-iterations",
            "-i",
            help="Number of optimization iterations to perform",
        ),
    ] = 1,
    record_steps: Annotated[
        bool,
        typer.Option(
            "--record-steps",
            "-s",
            help="Whether to record intermediate steps of the optimization (more images will be generated)",
        ),
    ] = False,
    num_points: Annotated[int, typer.Option("--num-points", "-p")] = 100,
    num_vertices: Annotated[int, typer.Option("--num-vertices", "-P")] = 20,
    noise_points: Annotated[float, typer.Option("--noise-points", "-n")] = 0.5,
    noise_polygon: Annotated[float, typer.Option("--noise-polygon", "-N")] = 1.0,
    radius_points: Annotated[float, typer.Option("--radius-points", "-r")] = 10.0,
    radius_polygon: Annotated[float, typer.Option("--radius-polygon", "-R")] = 10.0,
    shift_polygon_center_x: Annotated[
        float, typer.Option("--shift-polygon-center-x", "-x")
    ] = 0.0,
    shift_polygon_center_y: Annotated[
        float, typer.Option("--shift-polygon-center-y", "-y")
    ] = 0.0,
    random_seed: Annotated[Optional[int], typer.Option("--random-seed", "-S")] = None,
    verbose_int: Annotated[
        int,
        typer.Option(
            "--verbose",
            "-v",
            count=True,
            help="Verbosity level (0=Error, 1=Warning, 2=Info, 3=Debug)",
        ),
    ] = 0,
) -> None:
    with LoggingContext(verbose=verbose_int):
        shift_polygon_center = Vector(shift_polygon_center_x, shift_polygon_center_y)
        points, polygon = example_circle(
            num_points=num_points,
            num_vertices=num_vertices,
            noise_points=noise_points,
            noise_polygon=noise_polygon,
            radius_points=radius_points,
            radius_polygon=radius_polygon,
            shift_polygon_center=shift_polygon_center,
            random_seed=random_seed,
        )

        # Recorders
        bounds_recorders = None

        all_lines = AllLines(
            lines=polygon.lines,
            prev_lines=[
                (i - 1) % len(polygon.lines) for i in range(len(polygon.lines))
            ],
            next_lines=[
                (i + 1) % len(polygon.lines) for i in range(len(polygon.lines))
            ],
            touching_lines=[[] for _ in polygon.lines],
        )

        recorder_iterations = PlotRecorder()
        recorder_iterations.capture(
            "initial",
            points=points,
            segments=all_lines.get_segments(),
            title="Initial state",
            bounds=bounds_recorders,
        )

        if record_steps:
            recorder_steps = PlotRecorder()
        else:
            recorder_steps = None
        if recorder_steps is not None:
            recorder_steps.capture(
                "initial",
                points=points,
                segments=all_lines.get_segments(),
                title="Initial state",
                bounds=bounds_recorders,
            )

        algo = EdgeShiftingAlgorithm(
            all_lines,
            LinearCriterion(
                points,
                [1.0] * len(points),
                max_distance=LINEAR_CRITERION_MAX_DISTANCE,
            ),
        )

        for step in tqdm.trange(
            1, optimization_iterations + 1, desc="Optimization steps"
        ):

            if recorder_steps is not None:
                optimization_callback = lambda i: recorder_steps.capture(
                    f"optimization_step_{step:{f'0{len(str(optimization_iterations))}d'}}_line_{i}",
                    points=points,
                    segments=all_lines.get_segments(),
                    title=f"After optimization step {step} (line {i})",
                    bounds=bounds_recorders,
                )
            else:
                optimization_callback = lambda i: None

            algo.optimize_all_lines(callback_after_optimization=optimization_callback)

            recorder_iterations.capture(
                f"optimization_step_{step:{f'0{len(str(optimization_iterations))}d'}}",
                points=points,
                segments=all_lines.get_segments(),
                title=f"After optimization step {step}",
                bounds=bounds_recorders,
            )

        recorder_iterations.save_all_combined(
            Path("images/optimization_iterations.png")
        )
        if recorder_steps is not None:
            recorder_steps.save_all_combined(Path("images/optimization_steps.png"))


def experiment_one_move(line_idx: int, shift: Vector) -> None:
    # Points
    center_points = Point(0, 0)
    radius_points = 10
    num_points = 100
    noise_points = 0.0

    # Polygon
    center_polygon = center_points + Vector(0, 1)
    radius_polygon = radius_points - 1
    num_vertices_polygon = 20
    noise_polygon = 0.0

    # Recorders
    bounds_recorders = (
        (center_points.x - radius_points - 2, center_points.y - radius_points - 2),
        (center_points.x + radius_points + 2, center_points.y + radius_points + 2),
    )
    bounds_recorders = None

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
    all_lines_copy = copy.deepcopy(all_lines)

    recorder = PlotRecorder()
    recorder.capture(
        "initial",
        points=points,
        segments=all_lines.get_segments(),
        title="Initial state",
        bounds=bounds_recorders,
    )

    def callback(line_idx: int, shifted_lines: dict[int, Line]) -> None:
        for i, line in shifted_lines.items():
            all_lines_copy.update_line(i, line)
        recorder.capture(
            f"line_{line_idx}_shifted",
            points=points,
            segments=all_lines_copy.get_segments(),
            title=f"After shifting line {line_idx}",
            bounds=bounds_recorders,
        )

    shifted_lines = all_lines.lines_to_move_if_line_is_shifted(
        idx=line_idx,
        shift_dir=shift.normalized(),
        shift_length=shift.length(),
        callback=callback,
    )
    for line_idx, new_line in shifted_lines.items():
        all_lines.update_line(line_idx, new_line)

    recorder.capture(
        f"final",
        points=points,
        segments=all_lines.get_segments(),
        title=f"After optimization",
        bounds=bounds_recorders,
    )

    recorder.save_all_combined(Path("images/experiment_one_move.png"))


@app.command(
    "one_move",
    help="Run a single edge shifting experiment with specified parameters.",
)
def one_move(
    line_idx: Annotated[
        int,
        typer.Option(
            "--line-idx",
            "-l",
            help="Index of the line to shift",
        ),
    ],
    shift_x: Annotated[
        float,
        typer.Option(
            "--shift-x",
            "-x",
            help="X component of the shift vector",
        ),
    ],
    shift_y: Annotated[
        float,
        typer.Option(
            "--shift-y",
            "-y",
            help="Y component of the shift vector",
        ),
    ],
    verbose_int: Annotated[
        int,
        typer.Option(
            "--verbose",
            "-v",
            count=True,
            help="Verbosity level (0=Error, 1=Warning, 2=Info, 3=Debug)",
        ),
    ] = 0,
):
    with LoggingContext(verbose=verbose_int):
        plt.set_loglevel(level="warning")
        shift = Vector(shift_x, shift_y)
        experiment_one_move(line_idx=line_idx, shift=shift)


if __name__ == "__main__":

    app()
    app()
    app()
