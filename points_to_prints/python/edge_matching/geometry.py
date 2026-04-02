from abc import ABC, abstractmethod
from typing import List, Optional, Sequence, Tuple

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.artist import Artist
from matplotlib.axes import Axes
from matplotlib.patches import ArrowStyle

from ..utils.result import Result
from .constants import (
    LINE_COLOR,
    LINE_WIDTH,
    POINT_COLOR,
    POINT_SIZE,
    POLYGON_COLOR,
    POLYGON_POINT_COLOR,
    POLYGON_POINT_SIZE,
    POLYGON_WIDTH,
    SEGMENT_COLOR,
    SEGMENT_POINT_COLOR,
    SEGMENT_POINT_SIZE,
    SEGMENT_WIDTH,
)


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
        return f"Point({self.x:.2f}, {self.y:.2f})"

    def plot(
        self,
        ax: Optional[Axes] = None,
        color: str = POINT_COLOR,
        size: int = POINT_SIZE,
    ) -> Sequence[Artist]:
        if ax is None:
            ax = plt.gca()
        return ax.plot(self.x, self.y, "o", markersize=size, color=color)

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
        return f"Vector({self.x:.2f}, {self.y:.2f})"

    def plot(
        self,
        ax: Optional[Axes] = None,
        base_point: Optional[Point] = None,
        color: str = LINE_COLOR,
        width: int = LINE_WIDTH,
    ) -> Sequence[Artist]:
        if base_point is None:
            raise ValueError("Base point must be provided to plot a vector")
        xs = [base_point.x, base_point.x + self.x]
        ys = [base_point.y, base_point.y + self.y]
        if ax is None:
            ax = plt.gca()
        return [
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
        ]

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


class BoundingBox:
    def __init__(self, min_x: float, min_y: float, max_x: float, max_y: float) -> None:
        self.min_x = min_x
        self.min_y = min_y
        self.max_x = max_x
        self.max_y = max_y

    def buffer(self, distance: float) -> "BoundingBox":
        return BoundingBox(
            self.min_x - distance,
            self.min_y - distance,
            self.max_x + distance,
            self.max_y + distance,
        )


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

    def point_at(self, t: float) -> Point:
        return self.start.on_line_to(self.end, t)

    def get_line(self) -> Line:
        return Line.from_points(self.start, self.end)

    def bounding_box(self) -> BoundingBox:
        return BoundingBox(self.min_x, self.min_y, self.max_x, self.max_y)

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
    ) -> Sequence[Artist]:
        xs = [self.start.x, self.end.x]
        ys = [self.start.y, self.end.y]
        if ax is None:
            ax = plt.gca()
        artists: Sequence[Artist] = []
        artists.extend(
            ax.plot(
                xs,
                ys,
                linewidth=width,
                color=color,
                marker="o" if point_color and point_size else None,
                markersize=point_size if point_color and point_size else None,
                markerfacecolor=point_color if point_color and point_size else None,
            )
        )
        if self.name is not None:
            text_point = self.start.on_line_to(self.end, 0.5)
            artists.append(
                ax.annotate(
                    self.name,
                    xy=(text_point.x, text_point.y),
                    xytext=(text_point.x, text_point.y),
                )
            )
        return artists


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
    ) -> Sequence[Artist]:
        if ax is None:
            ax = plt.gca()

        artists: Sequence[Artist] = []

        for i in range(len(self.lines)):
            start_point = self.get_point(i)
            end_point = self.get_point((i + 1) % len(self.lines))
            artists.extend(
                Segment(start_point, end_point).plot(ax=ax, color=color, width=width)
            )

        if point_color and point_size:
            for i in range(len(self.lines)):
                artists.extend(
                    self.get_point(i).plot(ax=ax, color=point_color, size=point_size)
                )

        return artists
