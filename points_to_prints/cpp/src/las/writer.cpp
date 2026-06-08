#include "writer.hpp"

#include <string>
#include <vector>

#include <pdal/DimUtil.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/SpatialReference.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/io/BufferReader.hpp>

void LasWriter::write(
    const std::string &filename,
    const std::vector<LASclassification::Value> &allowed_classes) {

    pdal::StageFactory factory;

    pdal::Stage *writer = factory.createStage("writers.las");
    pdal::Options writer_opts;
    writer_opts.add("filename", filename);
    writer_opts.add("extra_dims", "all");
    writer->setOptions(writer_opts);

    writer->setSpatialReference(points->spatial_reference());

    pdal::BufferReader reader;
    reader.addView(points->get_view());

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

    writer->prepare(points->get_table());
    writer->execute(points->get_table());
}

std::string LasWriter::get_class_limits(
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
