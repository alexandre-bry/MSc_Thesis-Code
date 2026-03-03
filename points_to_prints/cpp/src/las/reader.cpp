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

#include "../geometry.hpp"
#include "../points.hpp"

void CustomLasReader::execute() {
    reader.prepare(table);
    auto view_set = reader.execute(table);
    view = *view_set.begin();
    dims = view->dims();
}

std::pair<std::vector<pdal::Dimension::Id>, std::vector<ProprietaryDimension>>
CustomLasReader::dimensions() const {
    pdal::Dimension::IdList pdal_dims = view->dims();
    std::vector<pdal::Dimension::Id> predefined_dims;
    std::vector<ProprietaryDimension> proprietary_dims;
    for (auto dim : pdal_dims) {
        if (static_cast<int>(dim) >= pdal::Dimension::PROPRIETARY) {
            proprietary_dims.push_back(
                ProprietaryDimension(view->dimName(dim), view->dimType(dim)));
        } else {
            predefined_dims.push_back(dim);
        }
    }
    return {predefined_dims, proprietary_dims};
}
pdal::LasHeader CustomLasReader::header() const { return reader.header(); }
unsigned int CustomLasReader::point_count() const {
    return reader.header().pointCount();
}
pdal::SpatialReference CustomLasReader::spatial_reference() const {
    return table.spatialReference();
}
pdal::PointViewPtr CustomLasReader::point_view() const { return view; }

OGREnvelopePtr CustomLasReader::bounding_box() const {
    pdal::BOX2D bounds;
    view->calculateBounds(bounds);
    OGREnvelope env;
    env.MinX = bounds.minx;
    env.MinY = bounds.miny;
    env.MaxX = bounds.maxx;
    env.MaxY = bounds.maxy;
    return std::make_unique<OGREnvelope>(env);
}

NewLasReader::NewLasReader(const std::string &filename) {
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