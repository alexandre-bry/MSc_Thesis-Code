#include "extension.h"
#include "geometries.h"

struct PlaneProjector {
    Vector_3 base_1;
    Vector_3 base_2;
    Vector_3 normal;
    Point_3 origin;

    PlaneProjector(const Triangle_3 &triangle) {
        const Plane_3 plane = triangle.supporting_plane();
        this->base_1 = plane.base1();
        this->base_1 /= CGAL::approximate_sqrt(this->base_1.squared_length());
        this->base_2 = plane.base2();
        this->base_2 /= CGAL::approximate_sqrt(this->base_2.squared_length());
        this->normal = plane.orthogonal_vector();
        this->normal /= CGAL::approximate_sqrt(this->normal.squared_length());
        this->origin = plane.point();
    }

    Point_2 to_2d(const Point_3 &point_3d) const {
        const auto translated = (point_3d - this->origin);
        const auto x = translated * this->base_1;
        const auto y = translated * this->base_2;
        const auto z = translated * this->normal;
        return {x, y};
    }

    Point_3 to_3d(const Point_2 &point_2d) const {
        const auto point_3d = this->origin + point_2d.x() * this->base_1 +
                              point_2d.y() * this->base_2;
        return point_3d;
    }
};

std::vector<Triangle_3> extend_triangle(const Triangle_3 &triangle,
                                        double distance) {
    const Plane_3 plane = triangle.supporting_plane();
    const PlaneProjector projector(triangle);
    const std::vector<Point_2> points = {projector.to_2d(triangle.vertex(0)),
                                         projector.to_2d(triangle.vertex(1)),
                                         projector.to_2d(triangle.vertex(2))};

    std::vector<Point_3> new_points(6);

    for (size_t i = 0; i < points.size(); i++) {
        auto p0 = points.at(i);
        auto p1 = points.at((i + 1) % points.size());
        auto p2 = points.at((i + 2) % points.size());

        // Get the normal pointing outwards
        const Vector_2 p01 = p1 - p0;
        Vector_2 normal(p01.y(), -p01.x());
        if ((p2 - p1) * normal > 0) {
            normal = -normal;
        }
        normal /= CGAL::approximate_sqrt(normal.squared_length());

        const Point_2 new_point_2d_0 = p0 + normal * distance;
        const Point_2 new_point_2d_1 = p1 + normal * distance;

        new_points[2 * i] = (projector.to_3d(new_point_2d_0));
        new_points[2 * i + 1] = (projector.to_3d(new_point_2d_1));
    }

    std::vector<Triangle_3> new_triangles{
        {new_points.at(0), new_points.at(2), new_points.at(4)},
        {new_points.at(0), new_points.at(1), new_points.at(2)},
        {new_points.at(2), new_points.at(3), new_points.at(4)},
        {new_points.at(4), new_points.at(5), new_points.at(0)},
    };

    return new_triangles;
}
