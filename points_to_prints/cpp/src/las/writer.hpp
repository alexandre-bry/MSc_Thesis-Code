#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/SpatialReference.hpp>

#include "../points.hpp"

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

        table.clearSpatialReferences();
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
               const std::vector<LASclassification::Value> &allowed_classes);

  private:
    std::vector<ProprietaryDimension> custom_dims;

    std::string get_class_limits(
        const std::vector<LASclassification::Value> &allowed_classes) const;
};

struct NewLasWriter {
  public:
    PtsStructs::StoragePtr points;

    NewLasWriter(std::vector<pdal::Dimension::Id> predefined_dims,
                 std::vector<ProprietaryDimension> proprietary_dims,
                 pdal::SpatialReference spatial_ref)
        : points(std::make_shared<PtsStructs::Storage>(
              predefined_dims, proprietary_dims, spatial_ref)) {}

    // Construct by sharing data from an existing Storage
    NewLasWriter(PtsStructs::StoragePtr storage) : points(storage) {}

    void write(const std::string &filename,
               const std::vector<LASclassification::Value> &allowed_classes);

  private:
    std::string get_class_limits(
        const std::vector<LASclassification::Value> &allowed_classes) const;
};