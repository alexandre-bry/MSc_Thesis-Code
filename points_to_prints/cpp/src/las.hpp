#pragma once

#include <cstddef>
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

    pdal::PointViewSet execute();
    pdal::LasHeader header() const;
    unsigned int pointCount() const;
    std::pair<std::vector<pdal::Dimension::Id>,
              std::vector<ProprietaryDimension>>
    dimensions() const;
    pdal::SpatialReference spatialReference() const;
    pdal::PointViewPtr pointView() const;

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
                    std::vector<ProprietaryDimension> proprietary_dims,
                    pdal::SpatialReference spatial_ref) {

        for (auto dim : predefined_dims) {
            table.layout()->registerDim(dim);
        }
        for (auto dim : proprietary_dims) {
            table.layout()->registerOrAssignDim(dim.name, dim.type);
        }

        table.setSpatialReference(spatial_ref);

        view = pdal::PointViewPtr(new pdal::PointView(table));
    }

    /**
     * @brief Returns the number of points currently in the writer.
     *
     * @return std::size_t
     */
    std::size_t pointCount();

    /**
     * @brief Set the value of an attribute for a point in the writer.
     *
     * @warning This fails if the given index is strictly higher than the
     * current point count.
     *
     * @tparam T The type of the value to set. Must match the type of the
     * dimension.
     * @param dim The dimension to set. Must be registered in the writer's
     * PointTable.
     * @param idx The index of the point to set the attribute for. Must be
     * between 0 and the current point count (inclusive).
     * @param value The value to set for the attribute. Must be of the same type
     * as the dimension.
     */
    template <typename T>
    void setField(pdal::Dimension::Id dim, pdal::PointId idx, const T value) {
        view->setField(dim, idx, value);
    }
    /**
     * @brief Set the value of an attribute for a point in the writer.
     *
     * @warning This fails if the given index is strictly higher than the
     * current point count.
     *
     * @tparam T The type of the value to set. Must match the type of the
     * dimension.
     * @param dim The dimension to set. Must be registered in the writer's
     * PointTable.
     * @param idx The index of the point to set the attribute for. Must be
     * between 0 and the current point count (inclusive).
     * @param value The value to set for the attribute. Must be of the same type
     * as the dimension.
     */
    template <typename T>
    void setField(ProprietaryDimension dim, pdal::PointId idx, const T value) {
        view->setField(table.layout()->findProprietaryDim(dim.name), idx,
                       value);
    }

    void write(const std::string &filename,
               const std::vector<LASclassification> &allowed_classes);

  private:
    std::vector<ProprietaryDimension> custom_dims;

    std::string get_class_limits(
        const std::vector<LASclassification> &allowed_classes) const;
};