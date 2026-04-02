import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Sequence, Tuple

import matplotlib.pyplot as plt
from matplotlib.artist import Artist
from matplotlib.axes import Axes

from .constants import *
from .geometry import Point, Polygon, Segment


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
        plt.axis("equal")
        plt.grid(True)

        if show:
            plt.show()

    def save(
        self,
        name: str,
        output_path: Path,
        dpi: int = 150,
        width: float = 5.0,
        height: float = 5.0,
    ) -> None:
        if name not in self._snapshots:
            raise ValueError(f"Unknown snapshot '{name}'")

        snap = self._snapshots[name]
        plt.figure(figsize=(width, height))

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
            num_rows,
            num_cols,
            figsize=(5 * num_cols, 5 * num_rows),
            sharex=True,
            sharey=True,
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
            ax.set_aspect("equal")
            ax.grid(True)

        # Hide unused subplots
        for idx in range(num_snapshots, len(axes)):
            axes[idx].set_visible(False)

        plt.tight_layout()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(output_path, dpi=dpi, bbox_inches="tight")
        plt.close()
        plt.close()
        plt.close()

    def save_combined_as_video(
        self, output_path: Path, fps: int = 10, dpi: int = 150
    ) -> None:
        """Save all snapshots as frames in a video."""
        import matplotlib.animation as animation

        names = self.snapshot_names()
        if not names:
            return

        # Create figure
        fig, ax = plt.subplots(figsize=(5, 5))

        artists_points: Sequence[Artist] = []
        artists_segments: Sequence[Artist] = []
        artists_polygons: Sequence[Artist] = []

        x_min = float("inf")
        x_max = float("-inf")
        y_min = float("inf")
        y_max = float("-inf")
        for name in names:
            snap = self._snapshots[name]
            for p in snap.points:
                x_min = min(x_min, p.x)
                x_max = max(x_max, p.x)
                y_min = min(y_min, p.y)
                y_max = max(y_max, p.y)
            for s in snap.segments:
                bbox = s.bounding_box()
                x_min = min(x_min, bbox.min_x)
                x_max = max(x_max, bbox.max_x)
                y_min = min(y_min, bbox.min_y)
                y_max = max(y_max, bbox.max_y)
            for poly in snap.polygons:
                for segment in poly.get_segments():
                    x_min = min(x_min, segment.start.x)
                    x_max = max(x_max, segment.end.x)
                    y_min = min(y_min, segment.start.y)
                    y_max = max(y_max, segment.end.y)

        def update(frame_idx: int) -> Sequence[Artist]:
            ax.clear()
            ax.set_xlim(x_min - 1, x_max + 1)
            ax.set_ylim(y_min - 1, y_max + 1)
            ax.set_aspect("equal")

            name = names[frame_idx]
            snap = self._snapshots[name]

            artists_points.clear()
            artists_segments.clear()
            artists_polygons.clear()

            for p in snap.points:
                artists_points.extend(
                    p.plot(
                        ax=ax, color=self.style.point_color, size=self.style.point_size
                    )
                )

            for s in snap.segments:
                artists_segments.extend(
                    s.plot(
                        ax=ax,
                        color=self.style.segment_color,
                        width=self.style.segment_width,
                        point_color=self.style.segment_point_color,
                        point_size=self.style.segment_point_size,
                    )
                )

            for poly in snap.polygons:
                artists_polygons.extend(
                    poly.plot(
                        ax=ax,
                        color=self.style.polygon_color,
                        width=self.style.polygon_width,
                        point_color=self.style.polygon_point_color,
                        point_size=self.style.polygon_point_size,
                    )
                )

            ax.set_title(snap.title or snap.name)

            return artists_points + artists_segments + artists_polygons

        anim = animation.FuncAnimation(fig, update, frames=len(names), repeat=False)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        anim.save(output_path, fps=fps, dpi=dpi)
        plt.close()
