#include "pca.hpp"

#include <CGAL/Classification/Local_eigen_analysis.h>
#include <CGAL/Classification/Point_set_neighborhood.h>
#include <eigen3/Eigen/Dense>
#include <pdal/Dimension.hpp>

#include "../geom/cgal.hpp"

std::tuple<Vector_3, Plane_3, EigenvaluesPCA_3>
compute_pca_once(const std::vector<Point_3> &points) {
    if (points.empty()) {
        throw std::invalid_argument(
            "compute_pca_once: points must not be empty");
    }

    // 1) Centroid
    Eigen::Vector3d centroid(0.0, 0.0, 0.0);
    for (const auto &p : points) {
        centroid +=
            Eigen::Vector3d(CGAL::to_double(p.x()), CGAL::to_double(p.y()),
                            CGAL::to_double(p.z()));
    }
    centroid /= static_cast<double>(points.size());

    // 2) Covariance matrix
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto &p : points) {
        Eigen::Vector3d d(CGAL::to_double(p.x()) - centroid.x(),
                          CGAL::to_double(p.y()) - centroid.y(),
                          CGAL::to_double(p.z()) - centroid.z());
        cov.noalias() += d * d.transpose();
    }
    cov /= static_cast<double>(points.size());

    // 3) Eigen decomposition of symmetric covariance
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    if (solver.info() != Eigen::Success) {
        throw std::runtime_error(
            "compute_pca_once: eigen decomposition failed");
    }

    // Eigen returns ascending eigenvalues: l0 <= l1 <= l2
    const auto vals = solver.eigenvalues();
    const auto vecs = solver.eigenvectors();

    // Smallest-eigenvalue eigenvector = normal of best-fit plane
    Eigen::Vector3d n = vecs.col(0).normalized();
    Vector_3 normal(n.x(), n.y(), n.z());

    // Plane through centroid with normal
    Point_3 c(centroid.x(), centroid.y(), centroid.z());
    Plane_3 plane(c, normal);

    EigenvaluesPCA_3 eigenvalues(vals(0), vals(1), vals(2));

    return {normal, plane, eigenvalues};
}

std::tuple<Vector_2, Line_2, EigenvaluesPCA_2>
compute_pca_once(const std::vector<Point_2> &points) {
    if (points.empty()) {
        throw std::invalid_argument(
            "compute_pca_once: points must not be empty");
    }

    // 1) Centroid
    Eigen::Vector2d centroid(0.0, 0.0);
    for (const auto &p : points) {
        centroid +=
            Eigen::Vector2d(CGAL::to_double(p.x()), CGAL::to_double(p.y()));
    }
    centroid /= static_cast<double>(points.size());

    // 2) Covariance matrix
    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
    for (const auto &p : points) {
        Eigen::Vector2d d(CGAL::to_double(p.x()) - centroid.x(),
                          CGAL::to_double(p.y()) - centroid.y());
        cov.noalias() += d * d.transpose();
    }
    cov /= static_cast<double>(points.size());

    // 3) Eigen decomposition of symmetric covariance
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
    if (solver.info() != Eigen::Success) {
        throw std::runtime_error(
            "compute_pca_once: eigen decomposition failed");
    }

    // Eigen returns ascending eigenvalues: l0 <= l1
    const auto vals = solver.eigenvalues();
    const auto vecs = solver.eigenvectors();

    // Smallest-eigenvalue eigenvector = normal of best-fit line
    Eigen::Vector2d n = vecs.col(0).normalized();
    Vector_2 normal(n.x(), n.y());

    // Line through centroid with normal
    Point_2 c(centroid.x(), centroid.y());
    Vector_2 dir(-normal.y(), normal.x()); // perpendicular to normal
    Line_2 line(c, dir);

    EigenvaluesPCA_2 eigenvalues(vals(0), vals(1));

    return {normal, line, eigenvalues};
}

void compute_pca(const std::vector<Point_3> &points,
                 std::vector<Vector_3> &normal_vectors,
                 std::vector<Plane_3> &tangent_planes,
                 std::vector<Eigenvalues> &eigenvalues) {
    Point_3_property_map point_map(points);
    Point_range indices;
    indices.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        indices.push_back(i);
    }
    unsigned int number_of_neighbours = 20;
    Neighborhood neighborhood(indices, point_map);
    Local_eigen local_eigen = Local_eigen::create_from_point_set(
        indices, point_map,
        neighborhood.k_neighbor_query(number_of_neighbours));

    normal_vectors.clear();
    tangent_planes.clear();
    eigenvalues.clear();
    normal_vectors.resize(points.size());
    tangent_planes.resize(points.size());
    eigenvalues.resize(points.size());

    for (std::size_t i = 0; i < points.size(); ++i) {
        normal_vectors[i] = local_eigen.normal_vector<K>(i);
        tangent_planes[i] = local_eigen.plane<K>(i);
        auto eigenvalues_i = local_eigen.eigenvalue(i);
        eigenvalues[i][0] = eigenvalues_i[0];
        eigenvalues[i][1] = eigenvalues_i[1];
        eigenvalues[i][2] = eigenvalues_i[2];
    }
}
