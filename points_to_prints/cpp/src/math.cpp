#include "math.hpp"

double CustomCGAL::angle_in_radians(const Vector_2 &u, const Vector_2 &v) {
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

double CustomCGAL::angle_in_degrees(const Vector_2 &u, const Vector_2 &v) {
    return angle_in_radians(u, v) * 180.0 / CGAL_PI;
}
