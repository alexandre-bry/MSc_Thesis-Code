#pragma once

#include "points.hpp"

struct Trajectory {
    std::map<double, Point3D> gps_time_to_point;

    Trajectory(const std::vector<Point3D> points,
               const std::vector<double> gps_times) {
        // Ensure the points and gps_times vectors have the same size
        if (points.size() != gps_times.size()) {
            throw std::runtime_error(
                "Points and GPS times vectors must have the same size");
        }

        if (points.empty()) {
            throw std::runtime_error("Trajectory cannot be empty");
        }
        if (points.size() == 1) {
            throw std::runtime_error("Trajectory must contain at least two "
                                     "points for interpolation");
        }

        for (size_t i = 0; i < points.size(); ++i) {
            gps_time_to_point[gps_times[i]] = points[i];
        }
    }

    Point3D get_point_at_gps_time(double gps_time) const {
        // Return the corresponding point if the GPS time exists
        auto it = gps_time_to_point.find(gps_time);
        if (it != gps_time_to_point.end()) {
            return it->second;
        }

        // Find the closest GPS times before and after the given GPS time
        auto it_after = gps_time_to_point.lower_bound(gps_time);
        auto it_before = (it_after == gps_time_to_point.begin())
                             ? gps_time_to_point.end()
                             : std::prev(it_after);

        if (it_before == gps_time_to_point.end()) {
            // The given GPS time is before the first GPS time in the trajectory

            auto it_first = gps_time_to_point.begin();
            auto it_second = std::next(it_first);

            double gps_time_first = it_first->first;
            double gps_time_second = it_second->first;
            Point3D point_first = it_first->second;
            Point3D point_second = it_second->second;

            // Linearly extrapolate the point at the given GPS time
            double t = (gps_time - gps_time_first) /
                       (gps_time_second - gps_time_first);
            return point_first * (1 - t) + point_second * t;

        } else if (it_after == gps_time_to_point.end()) {
            // The given GPS time is after the last GPS time in the trajectory

            auto it_last = std::prev(gps_time_to_point.end());
            auto it_second_last = std::prev(it_last);

            double gps_time_last = it_last->first;
            double gps_time_second_last = it_second_last->first;
            Point3D point_last = it_last->second;
            Point3D point_second_last = it_second_last->second;

            // Linearly extrapolate the point at the given GPS time
            double t = (gps_time - gps_time_second_last) /
                       (gps_time_last - gps_time_second_last);
            return point_second_last * (1 - t) + point_last * t;

        } else {
            // The given GPS time is between two GPS times in the trajectory
            double gps_time_before = it_before->first;
            double gps_time_after = it_after->first;
            Point3D point_before = it_before->second;
            Point3D point_after = it_after->second;

            // Linearly interpolate the point at the given GPS time
            double t = (gps_time - gps_time_before) /
                       (gps_time_after - gps_time_before);
            return point_before * (1 - t) + point_after * t;
        }
    }
};

Trajectory read_trajectory(const std::string &input_file);