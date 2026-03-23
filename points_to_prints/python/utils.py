from typing import List

from pyproj import Transformer


class Point2154:
    x: int
    y: int
    lon_4326: float
    lat_4326: float

    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y
        self._compute_epsg_4326()

    def _compute_epsg_4326(self):
        transformer = Transformer.from_crs("EPSG:2154", "EPSG:4326", always_xy=True)
        lon, lat = transformer.transform(self.x, self.y)
        self.lon_4326 = lon
        self.lat_4326 = lat

    def __add__(self, other):
        if not isinstance(other, Point2154):
            return NotImplemented
        return Point2154(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        if not isinstance(other, Point2154):
            return NotImplemented
        return Point2154(self.x - other.x, self.y - other.y)

    def __floordiv__(self, other):
        if not isinstance(other, int):
            return NotImplemented
        return Point2154(self.x // other, self.y // other)


class Box2154:
    p_min: Point2154
    p_max: Point2154
    p_center: Point2154

    def __init__(self, p_min: Point2154, p_max: Point2154):
        self.p_min = p_min
        self.p_max = p_max
        self.p_center = (self.p_min + self.p_max) // 2

    def get_tiles_boxes(
        self, tile_size: int = 1000, tile_base_point: Point2154 = Point2154(0, 0)
    ) -> List[Box2154]:
        start = (self.p_min - tile_base_point) // tile_size
        end = (self.p_max - tile_base_point - Point2154(1, 1)) // tile_size
        tiles_boxes = []
        for x in range(start.x, end.x + 1):
            for y in range(start.y, end.y + 1):
                tile_min = tile_base_point + Point2154(x * tile_size, y * tile_size)
                tile_max = tile_min + Point2154(tile_size, tile_size)
                tiles_boxes.append(Box2154(tile_min, tile_max))
        return tiles_boxes

    def __str__(self) -> str:
        return f"Box2154(p_min=({self.p_min.x}, {self.p_min.y}), p_max=({self.p_max.x}, {self.p_max.y}))"
