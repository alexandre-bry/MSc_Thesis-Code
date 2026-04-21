#pragma once

#include <vector>

#include <CGAL/Bbox_2.h>
#include <CGAL/Fuzzy_iso_box.h>
#include <CGAL/Fuzzy_sphere.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_2.h>
#include <CGAL/Search_traits_3.h>

#include "cgal.hpp"

typedef CGAL::Search_traits_2<K> Traits_2_base;
typedef CGAL::Search_traits_adapter<std::size_t, Point_2_property_map,
                                    Traits_2_base>
    Traits_2;
typedef CGAL::Kd_tree<Traits_2> Kd_tree_2;
typedef CGAL::Fuzzy_iso_box<Traits_2> Fuzzy_iso_box_2;
typedef Kd_tree_2::Tree Tree_2;
typedef Tree_2::Splitter Splitter_2;

struct KdTree_2 {
    std::vector<Point_2> points;
    Point_2_property_map point_map;
    Tree_2 tree;

    KdTree_2()
        : points(), point_map(points), tree(Splitter_2(), Traits_2(point_map)) {
    }

    KdTree_2(const std::vector<Point_2> &pts)
        : points(pts), point_map(points),
          tree(Splitter_2(), Traits_2(point_map)) {
        std::vector<std::size_t> indices;
        indices.reserve(points.size());
        for (std::size_t i = 0; i < points.size(); ++i) {
            indices.push_back(i);
        }
        tree.insert(indices.begin(), indices.end());
        tree.build();
    }

    void search_indices_in_box(const Point_2 &min_corner,
                               const Point_2 &max_corner,
                               std::vector<std::size_t> &result) const {
        Fuzzy_iso_box_2 box(min_corner, max_corner, 0.0, Traits_2(point_map));
        tree.search(std::back_inserter(result), box);
    }

    void search_indices_in_box(Bbox_2 bbox, double buffer_distance,
                               std::vector<std::size_t> &result) const {
        Point_2 min_corner(bbox.xmin() - buffer_distance,
                           bbox.ymin() - buffer_distance);
        Point_2 max_corner(bbox.xmax() + buffer_distance,
                           bbox.ymax() + buffer_distance);
        search_indices_in_box(min_corner, max_corner, result);
    }

    void search_points_in_box(const Point_2 &min_corner,
                              const Point_2 &max_corner,
                              std::vector<Point_2> &result) const {
        std::vector<std::size_t> indices;
        search_indices_in_box(min_corner, max_corner, indices);
        result.reserve(indices.size());
        for (std::size_t idx : indices) {
            result.push_back(points[idx]);
        }
    }

    void search_indices_in_circle(const Point_2 &center, double radius,
                                  std::vector<std::size_t> &result) const {
        CGAL::Fuzzy_sphere<Traits_2> sphere(center, radius, 0.0,
                                            Traits_2(point_map));
        tree.search(std::back_inserter(result), sphere);
    }

    void search_points_in_circle(const Point_2 &center, double radius,
                                 std::vector<Point_2> &result) const {
        std::vector<std::size_t> indices;
        search_indices_in_circle(center, radius, indices);
        result.reserve(indices.size());
        for (std::size_t idx : indices) {
            result.push_back(points[idx]);
        }
    }
};

typedef CGAL::Search_traits_3<K> Traits_3_base;
typedef CGAL::Search_traits_adapter<std::size_t, Point_3_property_map,
                                    Traits_3_base>
    Traits_3;
typedef CGAL::Kd_tree<Traits_3> Kd_tree_3;
typedef CGAL::Fuzzy_iso_box<Traits_3> Fuzzy_iso_box_3;
typedef Kd_tree_3::Tree Tree_3;
typedef Tree_3::Splitter Splitter_3;

struct KdTree_3 {
    std::vector<Point_3> points;
    Point_3_property_map point_map;
    Tree_3 tree;

    KdTree_3(const std::vector<Point_3> &pts)
        : points(pts), point_map(points),
          tree(Splitter_3(), Traits_3(point_map)) {
        std::vector<std::size_t> indices;
        indices.reserve(points.size());
        for (std::size_t i = 0; i < points.size(); ++i) {
            indices.push_back(i);
        }
        tree.insert(indices.begin(), indices.end());
        tree.build();
    }

    void search_indices_in_box(const Point_3 &min_corner,
                               const Point_3 &max_corner,
                               std::vector<std::size_t> &result) const {
        Fuzzy_iso_box_3 box(min_corner, max_corner, 0.0, Traits_3(point_map));
        tree.search(std::back_inserter(result), box);
    }

    void search_points_in_box(const Point_3 &min_corner,
                              const Point_3 &max_corner,
                              std::vector<Point_3> &result) const {
        std::vector<std::size_t> indices;
        search_indices_in_box(min_corner, max_corner, indices);
        result.reserve(indices.size());
        for (std::size_t idx : indices) {
            result.push_back(points[idx]);
        }
    }

    void search_indices_in_sphere(const Point_3 &center, double radius,
                                  std::vector<std::size_t> &result) const {
        CGAL::Fuzzy_sphere<Traits_3> sphere(center, radius, 0.0,
                                            Traits_3(point_map));
        tree.search(std::back_inserter(result), sphere);
    }

    void search_points_in_sphere(const Point_3 &center, double radius,
                                 std::vector<Point_3> &result) const {
        std::vector<std::size_t> indices;
        search_indices_in_sphere(center, radius, indices);
        result.reserve(indices.size());
        for (std::size_t idx : indices) {
            result.push_back(points[idx]);
        }
    }
};

typedef CGAL::Orthogonal_k_neighbor_search<Traits_3> Neighbor_search;
typedef Neighbor_search::Tree Tree_NN_3;

struct Search_NN_3 {
    std::vector<Point_3> points;
    Point_3_property_map point_map;
    Tree_NN_3 tree;

    Search_NN_3(const std::vector<Point_3> &pts)
        : points(pts), point_map(points),
          tree(Splitter_3(), Traits_3(point_map)) {
        std::vector<std::size_t> indices;
        indices.reserve(points.size());
        for (std::size_t i = 0; i < points.size(); ++i) {
            indices.push_back(i);
        }
        tree.insert(indices.begin(), indices.end());
        tree.build();
    }

    void search_indices_knn(const Point_3 &query, int k,
                            std::vector<std::size_t> &result) const {
        Neighbor_search search(tree, query, k);
        for (auto it = search.begin(); it != search.end(); ++it) {
            result.push_back(it->first);
        }
    }

    void search_points_knn(const Point_3 &query, int k,
                           std::vector<Point_3> &result) const {
        std::vector<std::size_t> indices;
        search_indices_knn(query, k, indices);
        result.reserve(indices.size());
        for (std::size_t idx : indices) {
            result.push_back(points[idx]);
        }
    }
};