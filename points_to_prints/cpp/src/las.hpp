#pragma once

#include <cstddef>
#include <iostream>
#include <pdal/DimUtil.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/SpatialReference.hpp>
#include <pdal/pdal_types.hpp>
#include <string>
#include <vector>

#include <pdal/Options.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/io/BufferReader.hpp>
#include <pdal/io/LasHeader.hpp>
#include <pdal/io/LasReader.hpp>

enum class LASclassification : uint8_t {
    Unclassified = 0,
    Unassigned = 1,
    Ground = 2,
    LowVegetation = 3,
    MediumVegetation = 4,
    HighVegetation = 5,
    Building = 6,
    LowPoint = 7,
    ModelKeyPoint = 8,
    Water = 9,
    Rail = 10,
    RoadSurface = 11,
    Overlap = 12
};

namespace CustomDimensions {
enum class Id {
    DownSignedVertGap,
    UpSignedVertGap,
    // MinSignedVertGap,
    // MaxSignedVertGap,
    IsRoofEdge,
    IsFootEdge,
    IsGenerated,
};
inline std::string name(Id id) {
    switch (id) {
    case Id::DownSignedVertGap:
        return "DownSignedVertGap";
    case Id::UpSignedVertGap:
        return "UpSignedVertGap";
    // case Id::MinSignedVertGap:
    //     return "MinSignedVertGap";
    // case Id::MaxSignedVertGap:
    //     return "MaxSignedVertGap";
    case Id::IsRoofEdge:
        return "IsRoofEdge";
    case Id::IsFootEdge:
        return "IsFootEdge";
    case Id::IsGenerated:
        return "IsGenerated";
    default:
        throw std::runtime_error("Unknown custom dimension ID");
    }
}

inline pdal::Dimension::Type type(Id id) {
    switch (id) {
    case Id::DownSignedVertGap:
        return pdal::Dimension::Type::Double;
    case Id::UpSignedVertGap:
        return pdal::Dimension::Type::Double;
    // case Id::MinSignedVertGap:
    //     return pdal::Dimension::Type::Double;
    // case Id::MaxSignedVertGap:
    //     return pdal::Dimension::Type::Double;
    case Id::IsRoofEdge:
        return pdal::Dimension::Type::Unsigned8;
    case Id::IsFootEdge:
        return pdal::Dimension::Type::Unsigned8;
    case Id::IsGenerated:
        return pdal::Dimension::Type::Unsigned8;
    default:
        throw std::runtime_error("Unknown custom dimension ID");
    }
}

} // namespace CustomDimensions

struct ProprietaryDimension {
    std::string name;
    pdal::Dimension::Type type;

    ProprietaryDimension(const std::string &n, pdal::Dimension::Type t)
        : name(n), type(t) {}

    ProprietaryDimension(const CustomDimensions::Id id)
        : name(CustomDimensions::name(id)), type(CustomDimensions::type(id)) {}
};

struct CustomLasReader {
  public:
    pdal::LasReader reader;
    pdal::PointTable table;
    pdal::PointViewSet view_set;
    pdal::PointViewPtr view;
    pdal::Dimension::IdList dims;

  public:
    CustomLasReader(const std::string &filename) : reader() {
        pdal::Options las_opts;
        las_opts.add("filename", filename);
        reader.setOptions(las_opts);
    }

    pdal::PointViewSet execute() {
        reader.prepare(table);
        view_set = reader.execute(table);
        view = *view_set.begin();
        dims = view->dims();
        return view_set;
    }

    pdal::LasHeader header() const { return reader.header(); }
    unsigned int pointCount() const { return reader.header().pointCount(); }
    std::pair<std::vector<pdal::Dimension::Id>,
              std::vector<ProprietaryDimension>>
    dimensions() const {
        pdal::Dimension::IdList pdal_dims = view->dims();
        std::vector<pdal::Dimension::Id> predefined_dims;
        std::vector<ProprietaryDimension> proprietary_dims;
        for (auto dim : pdal_dims) {
            if (static_cast<int>(dim) >= pdal::Dimension::PROPRIETARY) {
                proprietary_dims.push_back(ProprietaryDimension(
                    view->dimName(dim), view->dimType(dim)));
            } else {
                predefined_dims.push_back(dim);
            }
        }
        return {predefined_dims, proprietary_dims};
    }
    pdal::SpatialReference spatialReference() const {
        return table.spatialReference();
    }
    pdal::PointViewPtr pointView() const { return view; }

    template <typename T>
    T getFieldAs(pdal::Dimension::Id dim, pdal::PointId idx) const {
        return view->getFieldAs<T>(dim, idx);
    }

    template <typename T>
    T getFieldAs(ProprietaryDimension dim, pdal::PointId idx) const {
        return view->getFieldAs<T>(table.layout()->findProprietaryDim(dim.name),
                                   idx);
    }
};

struct CustomLasWriter {
    pdal::PointViewPtr view;
    pdal::PointTable table;

    CustomLasWriter(std::vector<pdal::Dimension::Id> predefined_dims,
                    std::vector<ProprietaryDimension> proprietary_dims) {

        for (auto dim : predefined_dims) {
            table.layout()->registerDim(dim);
        }
        for (auto dim : proprietary_dims) {
            table.layout()->registerOrAssignDim(dim.name, dim.type);
        }

        view = pdal::PointViewPtr(new pdal::PointView(table));
    }

    std::size_t pointCount() { return view->size(); }

    template <typename T>
    void setField(pdal::Dimension::Id dim, pdal::PointId idx, const T value) {
        view->setField(dim, idx, value);
    }

    template <typename T>
    void setField(ProprietaryDimension dim, pdal::PointId idx, const T value) {
        view->setField(table.layout()->findProprietaryDim(dim.name), idx,
                       value);
    }

    void write(const std::string &filename,
               const std::vector<LASclassification> &allowed_classes) {

        pdal::StageFactory factory;

        // std::cout << "Preparing filter and writer..." << std::endl;
        pdal::Stage *writer = factory.createStage("writers.las");
        // std::cout << "Preparing writer..." << std::endl;
        pdal::Options writer_opts;
        writer_opts.add("filename", filename);
        writer_opts.add("extra_dims", "all");
        writer->setOptions(writer_opts);

        pdal::BufferReader reader;
        reader.addView(this->view);

        if (allowed_classes.empty()) {
            // std::cout << "No class filtering applied." << std::endl;
            // Prepare the writer without filter
            writer->setInput(reader);
        } else {
            // Prepare the filter
            std::string class_limits = get_class_limits(allowed_classes);
            // std::cout << "Class limits: " << class_limits << std::endl;
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
        // std::cout << "Finished writing output LAS file: " << filename
        //           << std::endl;
    }

  private:
    std::vector<ProprietaryDimension> custom_dims;

    std::string get_class_limits(
        const std::vector<LASclassification> &allowed_classes) const {
        std::string class_limits = "";
        for (const auto &cls : allowed_classes) {
            if (!class_limits.empty()) {
                class_limits += ",";
            }
            std::string cls_number = std::to_string(static_cast<uint8_t>(cls));
            class_limits +=
                "Classification[" + cls_number + ":" + cls_number + "]";
        }
        return class_limits;
    }
};