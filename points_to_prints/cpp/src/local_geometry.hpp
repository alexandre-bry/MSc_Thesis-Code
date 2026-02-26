#pragma once

#include <array>
#include <cmath>

/**
 * @file local_geometry.hpp
 * @brief Compute local geometric properties from eigenvalues
 *
 * These features are commonly used in point cloud analysis and classification.
 * Assumes eigenvalues are sorted: λ1 ≤ λ2 ≤ λ3
 */

/**
 * @brief Compute planarity from eigenvalues
 * Measures how well points are distributed in a plane
 * Range: [0, 1], where 1 indicates perfect planarity
 *
 * Formula: (λ2 - λ1) / λ3
 */
inline double compute_planarity(double lambda1, double lambda2,
                                double lambda3) {
    if (lambda3 == 0)
        return 0.0;
    return (lambda2 - lambda1) / lambda3;
}

/**
 * @brief Compute linearity from eigenvalues
 * Measures how well points are distributed along a line
 * Range: [0, 1], where 1 indicates perfect linearity
 *
 * Formula: (λ3 - λ2) / λ3
 */
inline double compute_linearity(double lambda1, double lambda2,
                                double lambda3) {
    if (lambda3 == 0)
        return 0.0;
    return (lambda3 - lambda2) / lambda3;
}

/**
 * @brief Compute sphericity from eigenvalues
 * Measures how uniformly distributed points are in all directions
 * Range: [0, 1], where 1 indicates a perfect sphere
 *
 * Formula: λ1 / λ3
 */
inline double compute_sphericity(double lambda1, double lambda2,
                                 double lambda3) {
    if (lambda3 == 0)
        return 0.0;
    return lambda1 / lambda3;
}

/**
 * @brief Compute omnivariance (geometric mean of eigenvalues)
 * Characterizes the overall spread of the point distribution
 *
 * Formula: (λ1 * λ2 * λ3)^(1/3)
 */
inline double compute_omnivariance(double lambda1, double lambda2,
                                   double lambda3) {
    double product = lambda1 * lambda2 * lambda3;
    if (product < 0)
        return 0.0;
    return std::pow(product, 1.0 / 3.0);
}

/**
 * @brief Compute anisotropy from eigenvalues
 * Measures how much the distribution deviates from being isotropic
 * Range: [0, 1], where 1 indicates highly anisotropic distribution
 *
 * Formula: (λ3 - λ1) / λ3
 */
inline double compute_anisotropy(double lambda1, double lambda2,
                                 double lambda3) {
    if (lambda3 == 0)
        return 0.0;
    return (lambda3 - lambda1) / lambda3;
}

/**
 * @brief Compute eigenentropy from eigenvalues
 * Measures disorder in the point distribution
 * Range: [0, ln(3)], where 0 indicates order and ln(3) indicates disorder
 *
 * Formula: -Σ(λi * ln(λi)) where Σ(λi) = 1
 */
inline double compute_eigenentropy(double lambda1, double lambda2,
                                   double lambda3) {
    double sum = lambda1 + lambda2 + lambda3;
    if (sum == 0)
        return 0.0;

    double l1 = lambda1 / sum;
    double l2 = lambda2 / sum;
    double l3 = lambda3 / sum;

    double entropy = 0.0;
    if (l1 > 0)
        entropy -= l1 * std::log(l1);
    if (l2 > 0)
        entropy -= l2 * std::log(l2);
    if (l3 > 0)
        entropy -= l3 * std::log(l3);

    return entropy;
}

/**
 * @brief Compute surface variation (curvature) from eigenvalues
 * Measures how much the local surface curvature varies
 * Range: [0, 1]
 *
 * Formula: λ1 / (λ1 + λ2 + λ3)
 */
inline double compute_surface_variation(double lambda1, double lambda2,
                                        double lambda3) {
    double sum = lambda1 + lambda2 + lambda3;
    if (sum == 0)
        return 0.0;
    return lambda1 / sum;
}

/**
 * @brief Compute vertical range (height variation) from eigenvalues
 * Useful for characterizing vertical structure in point clouds
 *
 * This is a normalized measure where the smallest eigenvalue represents
 * the direction with least variation (normal to plane)
 */
inline double compute_verticality(double lambda1, double lambda2,
                                  double lambda3) {
    if (lambda3 == 0)
        return 0.0;
    // Lower verticality means more concentrated around horizontal plane
    return lambda1 / (lambda1 + lambda2);
}

/**
 * @brief Structure-based classification
 * Returns a simple classification based on eigenvalue relationships
 */
enum class LocalGeometryClass {
    Unclassified = 0,
    Linear = 1,    // λ3 >> λ2 ≈ λ1
    Planar = 2,    // λ2 >> λ1, λ3 ≈ λ2
    Volumetric = 3 // λ1 ≈ λ2 ≈ λ3 (spherical)
};

inline LocalGeometryClass
classify_local_geometry(double lambda1, double lambda2, double lambda3) {
    if (lambda3 == 0)
        return LocalGeometryClass::Unclassified;

    double linearity = (lambda3 - lambda2) / lambda3;
    double planarity = (lambda2 - lambda1) / lambda3;
    double sphericity = lambda1 / lambda3;

    // Thresholds (can be adjusted)
    const double linear_threshold = 0.5;
    const double planar_threshold = 0.5;
    const double spheric_threshold = 0.1;

    if (linearity > linear_threshold) {
        return LocalGeometryClass::Linear;
    } else if (planarity > planar_threshold) {
        return LocalGeometryClass::Planar;
    } else if (sphericity > spheric_threshold) {
        return LocalGeometryClass::Volumetric;
    }

    return LocalGeometryClass::Unclassified;
}
