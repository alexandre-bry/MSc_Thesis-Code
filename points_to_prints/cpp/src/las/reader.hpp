#pragma once

#include <memory>
#include <string>

#include <pdal/DimUtil.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/Options.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/SpatialReference.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/io/BufferReader.hpp>
#include <pdal/io/LasHeader.hpp>
#include <pdal/io/LasReader.hpp>
#include <pdal/pdal_types.hpp>

#include "../geom/points.hpp"

struct LasReader {
  public:
    pdal::LasReader reader;
    std::shared_ptr<pdal::PointTable> table;
    pdal::PointViewPtr view;
    PtsStructs::StoragePtr points;

    LasReader(const std::string &filename);
};