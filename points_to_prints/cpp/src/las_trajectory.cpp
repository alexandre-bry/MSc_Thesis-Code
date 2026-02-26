#include "las_trajectory.hpp"

#include <string>

#include "points.hpp"

Trajectory read_trajectory(const std::string &input_file) {
    // Parse the space-delimited text file
    std::ifstream infile(input_file);
    if (!infile.is_open()) {
        throw std::runtime_error("Could not open file: " + input_file);
    }

    std::vector<Point3D> points;
    std::vector<double> gps_times;
    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        double x, y, z;
        double gps_time;

        if (!(iss >> gps_time >> x >> y >> z)) {
            throw std::runtime_error("Error parsing line: " + line);
        }
        std::cout << "Read trajectory point: GPS Time = " << std::fixed
                  << gps_time << ", X = " << x << ", Y = " << y << ", Z = " << z
                  << std::endl;
        points.emplace_back(Point3D{x, y, z});
        gps_times.push_back(gps_time);
    }

    return Trajectory(points, gps_times);
}