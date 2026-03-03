#pragma once

#include <cstddef>
#include <memory>
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

#include "../geometry.hpp"
#include "../points.hpp"

struct CustomLasReader {
  public:
    pdal::LasReader reader;
    pdal::PointTable table;
    pdal::PointViewPtr view;
    pdal::Dimension::IdList dims;

  public:
    CustomLasReader(const std::string &filename) : reader() {
        pdal::Options las_opts;
        las_opts.add("filename", filename);
        reader.setOptions(las_opts);
    }

    void execute();
    pdal::LasHeader header() const;
    unsigned int point_count() const;
    std::pair<std::vector<pdal::Dimension::Id>,
              std::vector<ProprietaryDimension>>
    dimensions() const;
    pdal::SpatialReference spatial_reference() const;
    pdal::PointViewPtr point_view() const;
    OGREnvelopePtr bounding_box() const;

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

struct NewLasReader {
  public:
    pdal::LasReader reader;
    std::shared_ptr<pdal::PointTable> table;
    pdal::PointViewPtr view;
    PtsStructs::StoragePtr points;

    NewLasReader(const std::string &filename);
};