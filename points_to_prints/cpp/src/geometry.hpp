#pragma once

#include <ogr_core.h>
#include <string>

#include <ogr_api.h>
#include <ogr_geometry.h>
#include <vector>

#include "points.hpp"

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
        throw std::runtime_error("Unknown outline sources ID");
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
using OGREnvelopePtr = std::unique_ptr<OGREnvelope>;

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

    OGREnvelopePtr bounding_box() const {
        OGREnvelope env;
        polygon->getEnvelope(&env);
        return std::make_unique<OGREnvelope>(env);
    }

    // Copy constructor
    PolygonZ(const PolygonZ &other) {
        OGRPolygon *polygon_copy =
            dynamic_cast<OGRPolygon *>(other.polygon->clone());
        if (!polygon_copy) {
            throw std::runtime_error("Failed to clone polygon");
        }
        polygon.reset(polygon_copy);
    }
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
    MultiPolygonZ(const PolygonZ &polygon) {
        multi_polygon.reset(new OGRMultiPolygon());
        if (multi_polygon->addGeometry(polygon.polygon.get()) != OGRERR_NONE) {
            throw std::runtime_error("Failed to add polygon to multipolygon");
        }
    }
    MultiPolygonZ(const std::vector<PolygonZ> &polygons_) {
        multi_polygon.reset(new OGRMultiPolygon());
        for (const auto &polygon : polygons_) {
            if (multi_polygon->addGeometry(polygon.polygon.get()) !=
                OGRERR_NONE) {
                throw std::runtime_error(
                    "Failed to add polygon to multipolygon");
            }
        }
    }

    // Copy constructor
    MultiPolygonZ(const MultiPolygonZ &other) {
        OGRMultiPolygon *multi_polygon_copy =
            dynamic_cast<OGRMultiPolygon *>(other.multi_polygon->clone());
        if (!multi_polygon_copy) {
            throw std::runtime_error("Failed to clone multipolygon");
        }
        multi_polygon.reset(multi_polygon_copy);
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
    MultiPolygonZWithAttributes(const PolygonZ &polygon, const std::string &id_,
                                const OutlineSource::Id outline_source_)
        : MultiPolygonZ(polygon), id(id_), outline_source(outline_source_) {}
    MultiPolygonZWithAttributes(const std::vector<PolygonZ> &polygons_,
                                const std::string &id_,
                                const OutlineSource::Id outline_source_)
        : MultiPolygonZ(polygons_), id(id_), outline_source(outline_source_) {}
    MultiPolygonZWithAttributes(
        const PolygonZWithAttributes &polygon_with_attributes)
        : MultiPolygonZ(polygon_with_attributes),
          id(polygon_with_attributes.id),
          outline_source(polygon_with_attributes.outline_source) {}

    PolygonZWithAttributes get_polygon_with_attributes(int index) const {
        OGRPolygon *polygon =
            dynamic_cast<OGRPolygon *>(multi_polygon->getGeometryRef(index));
        if (!polygon) {
            throw std::runtime_error("Failed to cast geometry to Polygon");
        }
        // Clone the geometry since getGeometryRef returns a non-owning pointer
        OGRPolygon *polygon_copy = dynamic_cast<OGRPolygon *>(polygon->clone());
        if (!polygon_copy) {
            throw std::runtime_error("Failed to clone polygon");
        }
        return PolygonZWithAttributes(OGRPolygonPtr(polygon_copy), id,
                                      outline_source);
    }

    std::vector<PolygonZWithAttributes> get_polygons_with_attributes() const {
        std::vector<PolygonZWithAttributes> polygons;
        for (int i = 0; i < multi_polygon->getNumGeometries(); i++) {
            polygons.push_back(get_polygon_with_attributes(i));
        }
        return polygons;
    }

    OGREnvelopePtr bounding_box() const {
        OGREnvelope env;
        multi_polygon->getEnvelope(&env);
        return std::make_unique<OGREnvelope>(env);
    }

    std::vector<OGREnvelopePtr> bounding_boxes() const {
        std::vector<OGREnvelopePtr> boxes(multi_polygon->getNumGeometries());
        for (int i = 0; i < multi_polygon->getNumGeometries(); i++) {
            PolygonZWithAttributes polygon = get_polygon_with_attributes(i);
            boxes[i] = polygon.bounding_box();
        }
        return boxes;
    }
};