#include "las.hpp"
#include "geometry.hpp"
#include "points.hpp"

#include <cstddef>
#include <memory>
#include <ogr_core.h>
#include <pdal/util/Bounds.hpp>
#include <string>
#include <vector>

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

pdal::PointViewSet CustomLasReader::execute() {
    reader.prepare(table);
    view_set = reader.execute(table);
    view = *view_set.begin();
    dims = view->dims();
    return view_set;
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

std::size_t CustomLasWriter::pointCount() { return view->size(); }

void CustomLasWriter::write(
    const std::string &filename,
    const std::vector<LASclassification::Value> &allowed_classes) {

    pdal::StageFactory factory;

    pdal::Stage *writer = factory.createStage("writers.las");
    pdal::Options writer_opts;
    writer_opts.add("filename", filename);
    writer_opts.add("extra_dims", "all");
    writer->setOptions(writer_opts);

    pdal::BufferReader reader;
    reader.addView(this->view);

    if (allowed_classes.empty()) {
        // Prepare the writer without filter
        writer->setInput(reader);
    } else {
        // Prepare the filter
        std::string class_limits = get_class_limits(allowed_classes);
        pdal::Options filter_opts;
        filter_opts.add("limits", class_limits);
        pdal::Stage *filter = factory.createStage("filters.range");
        filter->setInput(reader);
        filter->setOptions(filter_opts);

        // Prepare the writer
        writer->setInput(*filter);
    }

    writer->prepare(table);
    writer->execute(table);
}

std::string CustomLasWriter::get_class_limits(
    const std::vector<LASclassification::Value> &allowed_classes) const {
    std::string class_limits = "";
    for (const auto &cls : allowed_classes) {
        if (!class_limits.empty()) {
            class_limits += ",";
        }
        std::string cls_number = std::to_string(static_cast<uint8_t>(cls));
        class_limits += "Classification[" + cls_number + ":" + cls_number + "]";
    }
    return class_limits;
}