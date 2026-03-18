#pragma once

#include <CGAL/Bbox_2.h>
#include <vector>

#include <CGAL/Fuzzy_iso_box.h>
#include <CGAL/Fuzzy_sphere.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Search_traits_2.h>
#include <CGAL/Search_traits_3.h>

#include "geometry.hpp"
#include "utils/cgal.hpp"

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

    std::vector<std::size_t>
    search_indices_in_box(const Point_2 &min_corner,
                          const Point_2 &max_corner) const {
        std::vector<std::size_t> result;
        Fuzzy_iso_box_2 box(min_corner, max_corner, 0.0, Traits_2(point_map));
        tree.search(std::back_inserter(result), box);
        return result;
    }

    std::vector<std::size_t>
    search_indices_in_box(const OGREnvelopePtr bbox,
                          double buffer_distance) const {
        std::vector<std::size_t> result;
        Point_2 min_corner(bbox->MinX - buffer_distance,
                           bbox->MinY - buffer_distance);
        Point_2 max_corner(bbox->MaxX + buffer_distance,
                           bbox->MaxY + buffer_distance);
        Fuzzy_iso_box_2 box(min_corner, max_corner, 0.0, Traits_2(point_map));
        tree.search(std::back_inserter(result), box);
        return result;
    }

    std::vector<std::size_t>
    search_indices_in_box(Bbox_2 bbox, double buffer_distance) const {
        std::vector<std::size_t> result;
        Point_2 min_corner(bbox.xmin() - buffer_distance,
                           bbox.ymin() - buffer_distance);
        Point_2 max_corner(bbox.xmax() + buffer_distance,
                           bbox.ymax() + buffer_distance);
        Fuzzy_iso_box_2 box(min_corner, max_corner, 0.0, Traits_2(point_map));
        tree.search(std::back_inserter(result), box);
        return result;
    }

    std::vector<Point_2> search_points_in_box(const Point_2 &min_corner,
                                              const Point_2 &max_corner) const {
        std::vector<std::size_t> indices =
            search_indices_in_box(min_corner, max_corner);
        std::vector<Point_2> result;
        result.reserve(indices.size());
        for (std::size_t idx : indices) {
            result.push_back(points[idx]);
        }
        return result;
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

    std::vector<std::size_t> search_indices_in_box(const Point_3 &min_corner,
                                                   const Point_3 &max_corner) {
        std::vector<std::size_t> result;
        Fuzzy_iso_box_3 box(min_corner, max_corner, 0.0, Traits_3(point_map));
        tree.search(std::back_inserter(result), box);
        return result;
    }

    std::vector<Point_3> search_points_in_box(const Point_3 &min_corner,
                                              const Point_3 &max_corner) {
        std::vector<std::size_t> indices =
            search_indices_in_box(min_corner, max_corner);
        std::vector<Point_3> result;
        result.reserve(indices.size());
        for (std::size_t idx : indices) {
            result.push_back(points[idx]);
        }
        return result;
    }
};