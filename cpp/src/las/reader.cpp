#include "reader.hpp"

#include <memory>
#include <vector>

#include <ogr_core.h>

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
#include <pdal/util/Bounds.hpp>

#include "../geom/points.hpp"

LasReader::LasReader(const std::string &filename) {
    table = std::make_shared<pdal::PointTable>();

    pdal::Options las_opts;
    las_opts.add("filename", filename);
    reader.setOptions(las_opts);
    reader.prepare(*table);
    auto view_set = reader.execute(*table);
    view = *view_set.begin();
    std::cout << "LAS file read successfully. Number of points: "
              << view->size() << std::endl;

    // Borrow table/view directly to avoid rebuilding the schema and copying
    // points.
    points = std::make_shared<PtsStructs::Storage>(view, table);
    std::cout << "Storage created successfully." << std::endl;
}