#include "pca.hpp"

#include <filesystem>
#include <iostream>

#include <CGAL/Classification/Local_eigen_analysis.h>
#include <CGAL/Classification/Point_set_neighborhood.h>
#include <eigen3/Eigen/Dense>
#include <pdal/Dimension.hpp>

#include "las/reader.hpp"
#include "las/writer.hpp"
#include "points.hpp"
#include "utils/cgal.hpp"

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

void add_pca(const std::string &input_file, const std::string &output_file,
             bool overwrite) {

    if (std::filesystem::exists(output_file) && !overwrite) {
        throw std::runtime_error("Output file already exists: " + output_file);
    }

    // Read the LAS file and get the point view
    std::cout << "Reading LAS file..." << std::endl;
    CustomLasReader las_reader(input_file);
    las_reader.execute();
    auto in_view = las_reader.point_view();
    auto [predefined_dims, proprietary_dims] = las_reader.dimensions();
    auto n_features = las_reader.point_count();

    std::cout << "Number of points: " << n_features << std::endl;

    Points3DWithAttributes points(in_view);

    std::cout << "Preparing output objects..." << std::endl;
    std::vector<pdal::Dimension::Id> pdal_dims = predefined_dims;
    std::vector<ProprietaryDimension> custom_dims = proprietary_dims;

    // Create ProprietaryDimension objects for all features
    pdal::Dimension::Id ev0_dim = pdal::Dimension::Id::Eigenvalue0;
    pdal::Dimension::Id ev1_dim = pdal::Dimension::Id::Eigenvalue1;
    pdal::Dimension::Id ev2_dim = pdal::Dimension::Id::Eigenvalue2;
    pdal::Dimension::Id planarity_dim = pdal::Dimension::Id::Planarity;
    pdal::Dimension::Id linearity_dim = pdal::Dimension::Id::Linearity;
    ProprietaryDimension sphericity_dim(CustomDimensions::Id::Sphericity);
    pdal::Dimension::Id omnivariance_dim = pdal::Dimension::Id::Omnivariance;
    pdal::Dimension::Id anisotropy_dim = pdal::Dimension::Id::Anisotropy;
    pdal::Dimension::Id eigenentropy_dim = pdal::Dimension::Id::Eigenentropy;
    pdal::Dimension::Id surface_variation_dim =
        pdal::Dimension::Id::SurfaceVariation;
    pdal::Dimension::Id verticality_dim = pdal::Dimension::Id::Verticality;

    std::vector<pdal::Dimension::Id> new_pdal_dims = {
        ev0_dim,        ev1_dim,          ev2_dim,
        planarity_dim,  linearity_dim,    omnivariance_dim,
        anisotropy_dim, eigenentropy_dim, surface_variation_dim,
        verticality_dim};
    std::vector<ProprietaryDimension> new_custom_dims = {sphericity_dim};
    pdal_dims.insert(pdal_dims.end(), new_pdal_dims.begin(),
                     new_pdal_dims.end());
    custom_dims.insert(custom_dims.end(), new_custom_dims.begin(),
                       new_custom_dims.end());
    CustomLasWriter las_writer(pdal_dims, custom_dims,
                               las_reader.spatial_reference());

    // Add the existing dimensions to the output point view
    // The points need to be processed in the order of the output view
    std::cout << "Adding existing dimensions to output view..." << std::endl;
    for (pdal::PointId out_idx = 0; out_idx < n_features; ++out_idx) {
        pdal::PointId in_idx = out_idx;
        for (pdal::Dimension::Id dim : predefined_dims) {
            double value = las_reader.getFieldAs<double>(dim, in_idx);
            las_writer.setField(dim, out_idx, value);
            // Get the value of the dimension in the correct type and set it
            // in the output view char *value; in_view->getField(value, dim,
            // pdal::Dimension::defaultType(dim),
            //                   in_idx);
            // las_writer.setField(dim, out_idx, value);
        }
        for (auto dim : proprietary_dims) {
            auto value = las_reader.getFieldAs<double>(dim, in_idx);
            las_writer.setField(dim, out_idx, value);
        }
    }

    // Compute PCA and add the eigenvalues to the output view
    std::cout << "Computing PCA..." << std::endl;
    std::vector<Point_3> cgal_points;
    cgal_points.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        const auto &point = points[i];
        cgal_points.emplace_back(point.x, point.y, point.z);
    }
    std::vector<Vector_3> normal_vectors;
    std::vector<Plane_3> tangent_planes;
    std::vector<Eigenvalues> eigenvalues;
    compute_pca(cgal_points, normal_vectors, tangent_planes, eigenvalues);

    std::cout << "Writing eigenvalues to output view..." << std::endl;

    for (std::size_t i = 0; i < points.size(); ++i) {
        // Write eigenvalues
        las_writer.setField(ev0_dim, i, eigenvalues[i][0]);
        las_writer.setField(ev1_dim, i, eigenvalues[i][1]);
        las_writer.setField(ev2_dim, i, eigenvalues[i][2]);

        // Compute and write geometric features
        auto features = compute_all_features(
            eigenvalues[i][0], eigenvalues[i][1], eigenvalues[i][2]);
        las_writer.setField(planarity_dim, i, features.planarity);
        las_writer.setField(linearity_dim, i, features.linearity);
        las_writer.setField(sphericity_dim, i, features.sphericity);
        las_writer.setField(omnivariance_dim, i, features.omnivariance);
        las_writer.setField(anisotropy_dim, i, features.anisotropy);
        las_writer.setField(eigenentropy_dim, i, features.eigenentropy);
        las_writer.setField(surface_variation_dim, i,
                            features.surface_variation);
        las_writer.setField(verticality_dim, i, features.verticality);
    }

    std::cout << "Writing output LAS file..." << std::endl;
    las_writer.write(output_file, {});

    std::cout << "Done." << std::endl;
}