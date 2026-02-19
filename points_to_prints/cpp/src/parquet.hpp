#pragma once

#include <string>

#include "arrow/status.h"

#include <ogr_api.h>
#include <ogr_geometry.h>

namespace OutlineSource {
enum class Id {
    AerialImagery,
    Cadastre,
    LiDARHD,
    Unknown,
};

inline std::string name(Id id) {
    switch (id) {
    case Id::AerialImagery:
        return "Imagerie aérienne";
    case Id::Cadastre:
        return "Cadastre";
    case Id::LiDARHD:
        return "LiDAR HD";
    case Id::Unknown:
        return "Unknown";
    default:
        throw std::runtime_error("Unknown custom dimension ID");
    }
}

inline Id from_string(const std::string &str) {
    if (str == name(Id::AerialImagery)) {
        return Id::AerialImagery;
    } else if (str == name(Id::Cadastre)) {
        return Id::Cadastre;
    } else if (str == name(Id::LiDARHD)) {
        return Id::LiDARHD;
    } else {
        return Id::Unknown;
    }
}

} // namespace OutlineSource

static OGRSpatialReference *getLAMB93() {
    static OGRSpatialReference srs;
    static bool initialized = false;
    if (!initialized) {
        srs.importFromEPSG(2154);
        initialized = true;
    }
    return &srs;
}

// Potentially interesting attributes in BD TOPO:
// - cleabs: unique identifier of the building footprint
// - nature: type of building that could allow to handle differently complex
// buildings (e.g. churches, windmills, towers, etc)
// - hauteur: height of the highest point of the gutter of the building
// - all the altitude_* attributes for the roof and the ground, which could be
// used as an indication of the roof points
// - origine_du_batiment: the source of the building outline
// - precision_altimetrique: the expected vertical precision of the building
// outline
// - precision_planimetrique: the expected horizontal precision of the building
// outline

using OGRGeometryPtr = std::unique_ptr<OGRGeometry>;
using OGRPolygonPtr = std::unique_ptr<OGRPolygon>;
using OGRMultiPolygonPtr = std::unique_ptr<OGRMultiPolygon>;

struct PolygonZ {
    OGRPolygonPtr polygon;

    PolygonZ(OGRPolygonPtr polygon_) : polygon(std::move(polygon_)) {}
    PolygonZ(OGRGeometryPtr geometry) {
        if (!geometry->IsSimple()) {
            throw std::runtime_error("Geometry is not simple");
        }
        if (geometry->getCoordinateDimension() != 3) {
            throw std::runtime_error("Geometry is not 3D");
        }
        if (!geometry->IsValid()) {
            throw std::runtime_error("Geometry is not valid");
        }

        polygon.reset(dynamic_cast<OGRPolygon *>(geometry.release()));
        if (!polygon) {
            throw std::runtime_error("Failed to cast geometry to Polygon");
        }
    };
};

struct PolygonZWithAttributes : PolygonZ {
    std::string id;
    OutlineSource::Id outline_source;

    PolygonZWithAttributes(OGRPolygonPtr geometry, const std::string &id_,
                           const OutlineSource::Id outline_source_)
        : PolygonZ(std::move(geometry)), id(id_),
          outline_source(outline_source_) {};
    PolygonZWithAttributes(OGRGeometryPtr geometry, const std::string &id_,
                           const OutlineSource::Id outline_source_)
        : PolygonZ(std::move(geometry)), id(id_),
          outline_source(outline_source_) {};
};

struct MultiPolygonZ {
    OGRMultiPolygonPtr multi_polygon;

    MultiPolygonZ() = default;
    MultiPolygonZ(OGRMultiPolygonPtr multi_polygon_)
        : multi_polygon(std::move(multi_polygon_)) {}
    MultiPolygonZ(OGRGeometryPtr geometry) {
        if (geometry->IsSimple()) {
            throw std::runtime_error("Geometry is simple");
        }
        if (geometry->getCoordinateDimension() != 3) {
            throw std::runtime_error("Geometry is not 3D");
        }
        if (!geometry->IsValid()) {
            throw std::runtime_error("Geometry is not valid");
        }

        multi_polygon.reset(
            dynamic_cast<OGRMultiPolygon *>(geometry.release()));
        if (!multi_polygon) {
            throw std::runtime_error("Failed to cast geometry to MultiPolygon");
        }
    }
    MultiPolygonZ(std::vector<PolygonZ> &&polygons_) {
        multi_polygon.reset(new OGRMultiPolygon());
        for (auto &polygon : polygons_) {
            if (multi_polygon->addGeometry(polygon.polygon.get()) !=
                OGRERR_NONE) {
                throw std::runtime_error(
                    "Failed to add polygon to multipolygon");
            }
        }
    }
};

struct MultiPolygonZWithAttributes : MultiPolygonZ {
    std::string id;
    OutlineSource::Id outline_source;

    MultiPolygonZWithAttributes() = default;
    MultiPolygonZWithAttributes(OGRMultiPolygonPtr multi_polygon_,
                                const std::string &id_,
                                const OutlineSource::Id outline_source_)
        : MultiPolygonZ(std::move(multi_polygon_)), id(id_),
          outline_source(outline_source_) {};
    MultiPolygonZWithAttributes(OGRGeometryPtr geometry, const std::string &id_,
                                const OutlineSource::Id outline_source_)
        : MultiPolygonZ(std::move(geometry)), id(id_),
          outline_source(outline_source_) {};
    MultiPolygonZWithAttributes(std::vector<PolygonZ> &&polygons_,
                                const std::string &id_,
                                const OutlineSource::Id outline_source_)
        : MultiPolygonZ(std::move(polygons_)), id(id_),
          outline_source(outline_source_) {}
};

struct TestParquetOptions {
    std::string input_file;
};

arrow::Status open_parquet(std::string &input_file);

struct ReadWriteBDTOPOOptions {
    std::string input_parquet_file;
    std::string output_parquet_file;
    bool overwrite;
};

arrow::Status read_building_outlines_from_bd_topo(
    const std::string &bd_topo_parquet_file,
    std::vector<MultiPolygonZWithAttributes> &outlines);

arrow::Status write_multi_polygons_to_parquet(
    const std::vector<MultiPolygonZWithAttributes> &multi_polygons,
    const std::string &output_file, bool overwrite);

arrow::Status read_write_bd_topo(const std::string &input_parquet_file,
                                 const std::string &output_parquet_file,
                                 bool overwrite);