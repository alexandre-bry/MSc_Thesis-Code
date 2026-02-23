#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

typedef K::Point_2 Point_2;
typedef K::Vector_2 Vector_2;
typedef K::Line_2 Line_2;

namespace CustomCGAL {
/**
 * @brief Return the angle in radians between two vectors, in the range [0, 2π]
 * radians.
 *
 * @param u
 * @param v
 * @return double
 */
double angle_in_radians(const Vector_2 &u, const Vector_2 &v);
/**
 * @brief Return the angle in degrees between two vectors, in the range [0, 360]
 * degrees.
 *
 * @param u
 * @param v
 * @return double
 */
double angle_in_degrees(const Vector_2 &u, const Vector_2 &v);
} // namespace CustomCGAL