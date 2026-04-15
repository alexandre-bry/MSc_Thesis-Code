import math
import random
from typing import List, Optional, Tuple

from .geometry import Point, Polygon, Segment, Vector


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


def sample_points_on_segments(
    segments: list[Segment], num_points: int, noise: float
) -> list[Point]:
    # Randomly sample points on the segments with a probability proportional to the segment length
    points = []

    total_length = sum(segment.length() for segment in segments)
    segments_probabilities = [segment.length() / total_length for segment in segments]

    chosen_segments = random.choices(
        population=list(range(len(segments))),
        weights=segments_probabilities,
        k=num_points,
    )
    points_per_segment = [0] * len(segments)
    for idx in chosen_segments:
        points_per_segment[idx] += 1

    for segment, num_points in zip(segments, points_per_segment):
        for i in range(num_points):
            t = -1
            while t < 0 or t > 1:
                t = i / num_points + random.uniform(0, 1 / num_points)
            point = segment.point_at(t)
            if noise > 0:
                point = point + segment.get_line().normal_vector * random.gauss(
                    0, noise
                )
            points.append(point)

    return points


def example_circle(
    num_points: int,
    num_vertices: int,
    noise_points: float,
    noise_polygon: float,
    radius_points: float,
    radius_polygon: float,
    shift_polygon_center: Vector,
    remove_segments: List[int],
    random_seed: Optional[int] = None,
) -> Tuple[list[Point], Polygon]:
    if random_seed is not None:
        random.seed(random_seed)

    center_polygon = Point(0, 0)

    polygon = generate_polygon_circle(
        center=center_polygon,
        radius=radius_polygon,
        num_vertices=num_vertices,
        noise=noise_polygon,
    )

    segments = polygon.get_segments()
    segments = [
        segment for i, segment in enumerate(segments) if i not in remove_segments
    ]

    points = sample_points_on_segments(
        segments=segments,
        num_points=num_points,
        noise=noise_points,
    )

    center_points = center_polygon - shift_polygon_center

    points = [
        center_points + (center_polygon.to(point)) * (radius_points / radius_polygon)
        for point in points
    ]

    return points, polygon
