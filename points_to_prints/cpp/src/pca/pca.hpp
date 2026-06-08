#pragma once

#include <array>

#include <CGAL/Classification/Local_eigen_analysis.h>
#include <CGAL/Classification/Point_set_neighborhood.h>
#include <CGAL/Point_set_3.h>

#include "../geom/cgal.hpp"
#include "local_geometry.hpp"

typedef CGAL::Classification::Point_set_neighborhood<K, Point_range,
                                                     Point_3_property_map>
    Neighborhood;
typedef CGAL::Classification::Local_eigen_analysis Local_eigen;
typedef std::array<double, 3> Eigenvalues;

struct EigenvaluesPCA_3 {
    double smallest;
    double middle;
    double largest;

    EigenvaluesPCA_3(double smallest_, double middle_, double largest_)
        : smallest(smallest_), middle(middle_), largest(largest_) {}
};
struct EigenvaluesPCA_2 {
    double smallest;
    double largest;

    EigenvaluesPCA_2(double smallest_, double largest_)
        : smallest(smallest_), largest(largest_) {}
};

std::tuple<Vector_3, Plane_3, EigenvaluesPCA_3>
compute_pca_once(const std::vector<Point_3> &points);
std::tuple<Vector_2, Line_2, EigenvaluesPCA_2>
compute_pca_once(const std::vector<Point_2> &points);

void compute_pca(const std::vector<Point_3> &points,
                 std::vector<Vector_3> &normal_vectors,
                 std::vector<Plane_3> &tangent_planes,
                 std::vector<Eigenvalues> &eigenvalues);
