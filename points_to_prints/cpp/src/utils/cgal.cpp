#include "cgal.hpp"

template <typename VectorType>
double _angle_in_radians(const VectorType &u, const VectorType &v) {
    double dot = u * v;
    double nu = std::sqrt(u.squared_length());
    double nv = std::sqrt(v.squared_length());

    double c = dot / (nu * nv);
    // Handle potential floating-point errors that may cause c to be slightly
    // outside [-1, 1]
    c = std::max(-1.0, std::min(1.0, c));

    double angle = std::acos(c);

    // Ensure the angle is in the range [0, 360]
    while (angle < 0) {
        angle += 2 * CGAL_PI;
    }
    while (angle >= 2 * CGAL_PI) {
        angle -= 2 * CGAL_PI;
    }
    return angle;
}

CustomCGAL::Angle CustomCGAL::angle(const Vector_2 &u, const Vector_2 &v) {
    double angle_rad = _angle_in_radians(u, v);
    return Angle::from_radians(angle_rad);
}

CustomCGAL::Angle CustomCGAL::angle(const Vector_3 &u, const Vector_3 &v) {
    double angle_rad = _angle_in_radians(u, v);
    return Angle::from_radians(angle_rad);
}

template <typename PointType, typename VectorType>
bool _are_almost_parallel(const VectorType &u, const VectorType &v,
                          CustomCGAL::Angle tolerance) {
    double angle_rad = _angle_in_radians(u, v);

    // Ensure the angle is in the range [0, 90]
    if (angle_rad > CGAL_PI) {
        angle_rad = 2 * CGAL_PI - angle_rad;
    }
    if (angle_rad > CGAL_PI / 2) {
        angle_rad = CGAL_PI - angle_rad;
    }
    return angle_rad <= tolerance.in_radians();
}

bool CustomCGAL::are_almost_parallel(const Vector_2 &u, const Vector_2 &v,
                                     Angle tolerance) {
    return _are_almost_parallel<Point_2, Vector_2>(u, v, tolerance);
}

bool CustomCGAL::are_almost_parallel(const Vector_3 &u, const Vector_3 &v,
                                     Angle tolerance) {
    return _are_almost_parallel<Point_3, Vector_3>(u, v, tolerance);
}

bool CustomCGAL::are_almost_collinear(const Point_2 &p1, const Point_2 &p2,
                                      const Point_2 &p3, Angle tolerance) {
    Vector_2 v1 = p2 - p1;
    Vector_2 v2 = p3 - p1;
    return are_almost_parallel(v1, v2, tolerance);
}

bool CustomCGAL::are_almost_collinear(const Point_3 &p1, const Point_3 &p2,
                                      const Point_3 &p3, Angle tolerance) {
    Vector_3 v1 = p2 - p1;
    Vector_3 v2 = p3 - p1;
    return are_almost_parallel(v1, v2, tolerance);
}