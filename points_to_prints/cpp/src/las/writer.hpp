#pragma once

#include <string>
#include <vector>

#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/SpatialReference.hpp>

#include "../geom/points.hpp"

struct LasWriter {
  public:
    PtsStructs::StoragePtr points;

    LasWriter(std::vector<pdal::Dimension::Id> predefined_dims,
              std::vector<ProprietaryDimension> proprietary_dims,
              pdal::SpatialReference spatial_ref)
        : points(std::make_shared<PtsStructs::Storage>(
              predefined_dims, proprietary_dims, spatial_ref)) {}

    // Construct by sharing data from an existing Storage
    LasWriter(PtsStructs::StoragePtr storage) : points(storage) {}

    void write(const std::string &filename,
               const std::vector<LASclassification::Value> &allowed_classes);

  private:
    std::string get_class_limits(
        const std::vector<LASclassification::Value> &allowed_classes) const;
};