#pragma once

#include <pdal/Dimension.hpp>

#include "../geom/cgal.hpp"

struct Trajectory {
  private:
    std::map<double, Point_3> gps_time_to_point;

  public:
    Trajectory(const std::vector<Point_3> points,
               const std::vector<double> gps_times);

    Point_3 get_pos_at_gps_time(double gps_time) const;
};

Trajectory read_trajectory(const std::string &input_file);