#pragma once

#include <array>

#include <CGAL/Classification/Local_eigen_analysis.h>
#include <CGAL/Classification/Point_set_neighborhood.h>
#include <CGAL/Point_set_3.h>

#include "local_geometry.hpp"
#include "utils/cgal.hpp"

typedef CGAL::Classification::Point_set_neighborhood<K, Point_range,
                                                     Point_3_property_map>
    Neighborhood;
typedef CGAL::Classification::Local_eigen_analysis Local_eigen;
typedef std::array<double, 3> Eigenvalues;

/**
 * @brief Structure to hold all computed local geometric features
 */
struct LocalGeometryFeatures {
    Eigenvalues eigenvalues;
    double planarity;
    double linearity;
    double sphericity;
    double omnivariance;
    double anisotropy;
    double eigenentropy;
    double surface_variation;
    double verticality;
};

struct EigenvaluesPCA {
    double smallest;
    double middle;
    double largest;

    EigenvaluesPCA(double smallest_, double middle_, double largest_)
        : smallest(smallest_), middle(middle_), largest(largest_) {}
};

std::tuple<Vector_3, Plane_3, EigenvaluesPCA>
compute_pca_once(const std::vector<Point_3> &points);

void compute_pca(const std::vector<Point_3> &points,
                 std::vector<Vector_3> &normal_vectors,
                 std::vector<Plane_3> &tangent_planes,
                 std::vector<Eigenvalues> &eigenvalues);

/**
 * @brief Compute all local geometric features from eigenvalues
 */
inline LocalGeometryFeatures
compute_all_features(double lambda1, double lambda2, double lambda3) {
    LocalGeometryFeatures features;
    features.eigenvalues[0] = lambda1;
    features.eigenvalues[1] = lambda2;
    features.eigenvalues[2] = lambda3;
    features.planarity = compute_planarity(lambda1, lambda2, lambda3);
    features.linearity = compute_linearity(lambda1, lambda2, lambda3);
    features.sphericity = compute_sphericity(lambda1, lambda2, lambda3);
    features.omnivariance = compute_omnivariance(lambda1, lambda2, lambda3);
    features.anisotropy = compute_anisotropy(lambda1, lambda2, lambda3);
    features.eigenentropy = compute_eigenentropy(lambda1, lambda2, lambda3);
    features.surface_variation =
        compute_surface_variation(lambda1, lambda2, lambda3);
    features.verticality = compute_verticality(lambda1, lambda2, lambda3);
    return features;
}

struct AddPCAOptions {
    std::string input_file;
    std::string output_file;
    bool overwrite = false;
};

void add_pca(const std::string &input_file, const std::string &output_file,
             bool overwrite);