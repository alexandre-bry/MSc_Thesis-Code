#include "trajectory.hpp"

#include <string>

Trajectory::Trajectory(const std::vector<Point_3> points,
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

Point_3 Trajectory::get_point_at_gps_time(double gps_time) const {
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
        Vector_3 point_first = it_first->second - CGAL::ORIGIN;
        Vector_3 point_second = it_second->second - CGAL::ORIGIN;

        // Linearly extrapolate the point at the given GPS time
        double t =
            (gps_time - gps_time_first) / (gps_time_second - gps_time_first);
        return CGAL::ORIGIN + point_first * (1 - t) + point_second * t;

    } else if (it_after == gps_time_to_point.end()) {
        // The given GPS time is after the last GPS time in the trajectory

        auto it_last = std::prev(gps_time_to_point.end());
        auto it_second_last = std::prev(it_last);

        double gps_time_last = it_last->first;
        double gps_time_second_last = it_second_last->first;
        Vector_3 point_last = it_last->second - CGAL::ORIGIN;
        Vector_3 point_second_last = it_second_last->second - CGAL::ORIGIN;

        // Linearly extrapolate the point at the given GPS time
        double t = (gps_time - gps_time_second_last) /
                   (gps_time_last - gps_time_second_last);
        return CGAL::ORIGIN + point_second_last * (1 - t) + point_last * t;

    } else {
        // The given GPS time is between two GPS times in the trajectory
        double gps_time_before = it_before->first;
        double gps_time_after = it_after->first;
        Vector_3 point_before = it_before->second - CGAL::ORIGIN;
        Vector_3 point_after = it_after->second - CGAL::ORIGIN;

        // Linearly interpolate the point at the given GPS time
        double t =
            (gps_time - gps_time_before) / (gps_time_after - gps_time_before);
        return CGAL::ORIGIN + point_before * (1 - t) + point_after * t;
    }
}

Trajectory read_trajectory(const std::string &input_file) {
    // Parse the space-delimited text file
    std::ifstream infile(input_file);
    if (!infile.is_open()) {
        throw std::runtime_error("Could not open file: " + input_file);
    }

    std::vector<Point_3> points;
    std::vector<double> gps_times;
    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        double x, y, z;
        double gps_time;

        if (!(iss >> gps_time >> x >> y >> z)) {
            throw std::runtime_error("Error parsing line: " + line);
        }
        points.emplace_back(Point_3{x, y, z});
        gps_times.push_back(gps_time);
    }

    return Trajectory(points, gps_times);
}