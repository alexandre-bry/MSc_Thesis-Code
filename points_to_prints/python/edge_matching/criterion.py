from abc import ABC, abstractmethod
from typing import List

from fastquadtree import QuadTree

from .constants import *
from .geometry import Point, Segment


class Criterion(ABC):

    @abstractmethod
    def evaluate_segments(
        self, segments: List[Segment], initial_segment_lengths: list[float]
    ) -> float:
        raise NotImplementedError("Subclasses must implement this method")


class LinearCriterion(Criterion):

    def __init__(
        self,
        points: list[Point],
        weights: list[float],
        max_distance: float,
        alpha_edge_difference: float,
        # alpha_abs: float,
    ) -> None:
        # Check that weights and points have the same length
        if len(points) != len(weights):
            raise ValueError("Points and weights must have the same length")

        self.points = points
        self.weights = weights
        self.max_distance = max_distance
        self.alpha_edge_difference = alpha_edge_difference
        # self.alpha_abs = alpha_abs

        min_x = min(point.x for point in points) - 1e-6
        max_x = max(point.x for point in points) + 1e-6
        min_y = min(point.y for point in points) - 1e-6
        max_y = max(point.y for point in points) + 1e-6
        self.qt = QuadTree((min_x, min_y, max_x, max_y), len(points))
        for i, point in enumerate(points):
            self.qt.insert((point.x, point.y), id_=i)

    def evaluate_segments(
        self, segments: List[Segment], initial_segment_lengths: list[float]
    ) -> float:
        edge_differences_value = 0.0
        points_best_proximity_score: List[float] = [0.0] * len(self.points)
        for segment, initial_length in zip(segments, initial_segment_lengths):
            bounding_box = segment.bounding_box().buffer(self.max_distance)
            ids, coords = self.qt.query_np(
                (
                    bounding_box.min_x,
                    bounding_box.min_y,
                    bounding_box.max_x,
                    bounding_box.max_y,
                )
            )

            for point_idx in ids:
                point = self.points[point_idx]
                weight = self.weights[point_idx]
                normal_distance, segment_distance = segment.distance_to_point(point)
                if normal_distance > self.max_distance:
                    continue
                if segment_distance > 0:
                    continue
                # proximity_value -= weight * (1.0 - normal_distance)
                points_best_proximity_score[point_idx] = max(
                    points_best_proximity_score[point_idx],
                    weight * (1.0 - normal_distance),
                )

            current_length = segment.length()
            edge_differences_value += abs(current_length - initial_length) ** 2

        proximity_value = -sum(points_best_proximity_score)
        edge_differences_value *= self.alpha_edge_difference

        # if perimeter > self.initial_perimeter:
        #     perimeter_ratio_value = self.alpha_ratio * abs(
        #         perimeter / self.initial_perimeter
        #     )
        # else:
        #     perimeter_ratio_value = self.alpha_abs * abs(
        #         self.initial_perimeter / perimeter
        #     )

        # perimeter_abs_value = self.alpha_abs * perimeter

        return proximity_value + edge_differences_value
