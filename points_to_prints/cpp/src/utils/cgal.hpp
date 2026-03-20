#pragma once

#include <cstddef>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <boost/property_map/vector_property_map.hpp>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

typedef K::Point_2 Point_2;
typedef K::Vector_2 Vector_2;
typedef K::Line_2 Line_2;
typedef K::Segment_2 Segment_2;
typedef CGAL::Bbox_2 Bbox_2;

class Point_2_property_map {
    const std::vector<Point_2> &points;

  public:
    typedef Point_2 value_type;
    typedef const value_type &reference;
    typedef std::size_t key_type;
    typedef boost::lvalue_property_map_tag category;

    Point_2_property_map(const std::vector<Point_2> &pts) : points(pts) {}

    reference operator[](key_type k) const { return points[k]; }

    friend reference get(const Point_2_property_map &ppmap, key_type i) {
        return ppmap[i];
    }
};

typedef K::Point_3 Point_3;
typedef K::Vector_3 Vector_3;
typedef K::Line_3 Line_3;
typedef K::Plane_3 Plane_3;
typedef K::Segment_3 Segment_3;

// Property map for Point_3
class Point_3_property_map {
    const std::vector<Point_3> *points;

  public:
    typedef Point_3 value_type;
    typedef const value_type &reference;
    typedef std::size_t key_type;
    typedef boost::lvalue_property_map_tag category;

    Point_3_property_map() : points(nullptr) {}
    Point_3_property_map(const std::vector<Point_3> &pts) : points(&pts) {}

    reference operator[](key_type k) const { return (*points)[k]; }

    friend reference get(const Point_3_property_map &ppmap, key_type i) {
        return ppmap[i];
    }
};

typedef std::vector<std::size_t> Point_range;

namespace CustomCGAL {

// Custom angle class to store angles in both radians and degrees and ensure
// they are always in the range [0, 360]
struct Angle {
  private:
    double radians;
    double degrees;

  public:
    static const Angle from_radians(double radians);
    static const Angle from_degrees(double degrees);

    double in_radians() const { return radians; }
    double in_degrees() const { return degrees; }

    Angle in_180() const;
};

Angle angle(const Vector_2 &u, const Vector_2 &v);
Angle angle(const Vector_3 &u, const Vector_3 &v);
Angle angle(const Point_2 &p, const Point_2 &q, const Point_2 &r);
Angle angle(const Point_3 &p, const Point_3 &q, const Point_3 &r);

bool are_almost_parallel(const Vector_2 &u, const Vector_2 &v, Angle tolerance);
bool are_almost_parallel(const Vector_3 &u, const Vector_3 &v, Angle tolerance);

// TODO: Look into the are_almost_collinear functions and see if the current
// implementation makes sense

/**
 * @brief Check if three points are collinear up to a certain tolerance.
 *
 * @param p1
 * @param p2
 * @param p3
 * @param tolerance: The maximum angle between the vectors (p2 - p1) and (p3
 * - p1) for the points to be considered collinear.
 * @return true if the points are collinear, false otherwise
 */
bool are_almost_collinear(const Point_2 &p1, const Point_2 &p2,
                          const Point_2 &p3, Angle tolerance);
/**
 * @brief Check if three points are collinear up to a certain tolerance.
 *
 * @param p1
 * @param p2
 * @param p3
 * @param tolerance: The maximum angle between the vectors (p2 - p1) and (p3 -
 * p1) for the points to be considered collinear.
 * @return true if the points are collinear, false otherwise
 */
bool are_almost_collinear(const Point_3 &p1, const Point_3 &p2,
                          const Point_3 &p3, Angle tolerance);

Point_2 intersection(const Line_2 &line1, const Line_2 &line2);
} // namespace CustomCGAL