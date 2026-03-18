#pragma once

#include <ogr_core.h>
#include <string>

#include <ogr_api.h>
#include <ogr_geometry.h>
#include <vector>

#include "utils/cgal.hpp"

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
using OGRMultiLineStringPtr = std::unique_ptr<OGRMultiLineString>;
using OGRPolygonPtr = std::unique_ptr<OGRPolygon>;
using OGRMultiPolygonPtr = std::unique_ptr<OGRMultiPolygon>;
using OGREnvelopePtr = std::unique_ptr<OGREnvelope>;

// Abstract base class for all geometries
class Geometry {
  public:
    virtual ~Geometry() = default;

    // Pure virtual functions that all geometries must implement
    virtual OGRGeometryPtr get_geom() const = 0;
    virtual OGREnvelopePtr bounding_box() const = 0;

    // Clone pattern for copying (since constructors can't be virtual)
    virtual std::unique_ptr<Geometry> clone() const = 0;
};

// Abstract base class for geometries with attributes
class GeometryWithAttributes : public virtual Geometry {
  public:
    virtual ~GeometryWithAttributes() = default;

    // Additional pure virtual functions for attributed geometries
    virtual std::string get_id() const = 0;
    virtual OutlineSource::Id get_outline_source() const = 0;
};

struct PolygonZ : public virtual Geometry {
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
    PolygonZ(const std::vector<Point_3> &points, bool first_is_repeated) {
        OGRPolygon *polygon = new OGRPolygon();
        OGRLinearRing *ring = new OGRLinearRing();
        for (const auto &p : points) {
            ring->addPoint(p.x(), p.y(), p.z());
        }
        // Close the ring by adding the first point at the end
        if (!first_is_repeated) {
            ring->addPoint(points[0].x(), points[0].y(), points[0].z());
        }
        polygon->addRing(ring);
        this->polygon.reset(polygon);
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

    OGRGeometryPtr get_geom() const override {
        OGRPolygon *polygon_copy = dynamic_cast<OGRPolygon *>(polygon->clone());
        if (!polygon_copy) {
            throw std::runtime_error("Failed to clone polygon");
        }
        return OGRGeometryPtr(polygon_copy);
    }

    OGREnvelopePtr bounding_box() const override {
        OGREnvelope env;
        polygon->getEnvelope(&env);
        return std::make_unique<OGREnvelope>(env);
    }

    std::unique_ptr<Geometry> clone() const override {
        return std::make_unique<PolygonZ>(*this);
    }
};

struct PolygonZWithAttributes : PolygonZ, GeometryWithAttributes {
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

    // Implement GeometryWithAttributes interface
    std::string get_id() const override { return id; }
    OutlineSource::Id get_outline_source() const override {
        return outline_source;
    }

    std::unique_ptr<Geometry> clone() const override {
        return std::make_unique<PolygonZWithAttributes>(*this);
    }
};

struct MultiPolygonZ : public virtual Geometry {
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

    OGRGeometryPtr get_geom() const override {
        OGRMultiPolygon *multi_polygon_copy =
            dynamic_cast<OGRMultiPolygon *>(multi_polygon->clone());
        if (!multi_polygon_copy) {
            throw std::runtime_error("Failed to clone multipolygon");
        }
        return OGRGeometryPtr(multi_polygon_copy);
    }

    OGREnvelopePtr bounding_box() const override {
        OGREnvelope env;
        multi_polygon->getEnvelope(&env);
        return std::make_unique<OGREnvelope>(env);
    }

    std::unique_ptr<Geometry> clone() const override {
        return std::make_unique<MultiPolygonZ>(*this);
    }
};

struct MultiPolygonZWithAttributes : MultiPolygonZ, GeometryWithAttributes {
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

    OGREnvelopePtr bounding_box() const override {
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

    // Implement GeometryWithAttributes interface
    std::string get_id() const override { return id; }
    OutlineSource::Id get_outline_source() const override {
        return outline_source;
    }

    std::unique_ptr<Geometry> clone() const override {
        return std::make_unique<MultiPolygonZWithAttributes>(*this);
    }
};

struct MultiLineStringZ : public virtual Geometry {
    OGRMultiLineStringPtr multi_line_string;

    MultiLineStringZ() = default;
    MultiLineStringZ(OGRMultiLineStringPtr multi_line_string_)
        : multi_line_string(std::move(multi_line_string_)) {}
    MultiLineStringZ(OGRGeometryPtr geometry) {
        if (geometry->IsSimple()) {
            throw std::runtime_error("Geometry is simple");
        }
        if (geometry->getCoordinateDimension() != 3) {
            throw std::runtime_error("Geometry is not 3D");
        }
        if (!geometry->IsValid()) {
            throw std::runtime_error("Geometry is not valid");
        }

        multi_line_string.reset(
            dynamic_cast<OGRMultiLineString *>(geometry.release()));
        if (!multi_line_string) {
            throw std::runtime_error(
                "Failed to cast geometry to MultiLineString");
        }
    }

    // Copy constructor
    MultiLineStringZ(const MultiLineStringZ &other) {
        OGRMultiLineString *multi_line_string_copy =
            dynamic_cast<OGRMultiLineString *>(
                other.multi_line_string->clone());
        if (!multi_line_string_copy) {
            throw std::runtime_error("Failed to clone multilinestring");
        }
        multi_line_string.reset(multi_line_string_copy);
    }

    void add_line(const Segment_3 &segment) {
        OGRLineString *line_string = new OGRLineString();
        line_string->addPoint(segment.point(0).x(), segment.point(0).y(),
                              segment.point(0).z());
        line_string->addPoint(segment.point(1).x(), segment.point(1).y(),
                              segment.point(1).z());

        if (!multi_line_string) {
            multi_line_string.reset(new OGRMultiLineString());
        }
        if (multi_line_string->addGeometry(line_string) != OGRERR_NONE) {
            throw std::runtime_error("Failed to add line to MultiLineString");
        }
    }

    OGRGeometryPtr get_geom() const override {
        OGRMultiLineString *multi_line_string_copy =
            dynamic_cast<OGRMultiLineString *>(multi_line_string->clone());
        if (!multi_line_string_copy) {
            throw std::runtime_error("Failed to clone multilinestring");
        }
        return OGRGeometryPtr(multi_line_string_copy);
    }

    OGREnvelopePtr bounding_box() const override {
        OGREnvelope env;
        multi_line_string->getEnvelope(&env);
        return std::make_unique<OGREnvelope>(env);
    }

    std::unique_ptr<Geometry> clone() const override {
        return std::make_unique<MultiLineStringZ>(*this);
    }
};

struct MultiLineStringZWithAttributes : MultiLineStringZ,
                                        GeometryWithAttributes {
    std::string id;
    OutlineSource::Id outline_source;

    MultiLineStringZWithAttributes() = default;
    MultiLineStringZWithAttributes(MultiLineStringZ multi_line_string_,
                                   const std::string &id_,
                                   const OutlineSource::Id outline_source_)
        : MultiLineStringZ(std::move(multi_line_string_)), id(id_),
          outline_source(outline_source_) {};
    MultiLineStringZWithAttributes(OGRMultiLineStringPtr multi_line_string_,
                                   const std::string &id_,
                                   const OutlineSource::Id outline_source_)
        : MultiLineStringZ(std::move(multi_line_string_)), id(id_),
          outline_source(outline_source_) {};
    MultiLineStringZWithAttributes(OGRGeometryPtr geometry,
                                   const std::string &id_,
                                   const OutlineSource::Id outline_source_)
        : MultiLineStringZ(std::move(geometry)), id(id_),
          outline_source(outline_source_) {}

    // Implement GeometryWithAttributes interface
    std::string get_id() const override { return id; }
    OutlineSource::Id get_outline_source() const override {
        return outline_source;
    }

    std::unique_ptr<Geometry> clone() const override {
        return std::make_unique<MultiLineStringZWithAttributes>(*this);
    }
};