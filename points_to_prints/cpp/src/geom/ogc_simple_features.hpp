#pragma once

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <arrow/api.h>
#include <ogr_api.h>
#include <ogr_core.h>
#include <ogr_geometry.h>

#include "cgal.hpp"

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

class Geometry {
  public:
    virtual ~Geometry() = default;

    virtual OGRGeometryPtr get_geom() const = 0;
    virtual OGREnvelopePtr bounding_box() const = 0;

    virtual std::unique_ptr<Geometry> clone() const = 0;
};

class GeometryWithAttributes : public virtual Geometry {
  public:
    virtual ~GeometryWithAttributes() = default;

    virtual std::string get_id() const = 0;
    virtual OutlineSource::Id get_outline_source() const = 0;
};

struct PolygonZ : public virtual Geometry {
    OGRPolygonPtr polygon;

    PolygonZ(OGRPolygonPtr polygon_);
    PolygonZ(OGRGeometryPtr geometry);
    PolygonZ(const std::vector<Point_3> &points, bool first_is_repeated);
    PolygonZ(const std::vector<std::vector<Point_3>> &rings,
             bool first_is_repeated);
    PolygonZ(const PolygonZ &other);

    OGRGeometryPtr get_geom() const override;
    OGREnvelopePtr bounding_box() const override;
    std::unique_ptr<Geometry> clone() const override;
};

struct PolygonZWithAttributes : PolygonZ, GeometryWithAttributes {
    std::string id;
    OutlineSource::Id outline_source;

    PolygonZWithAttributes(OGRPolygonPtr geometry, const std::string &id_,
                           const OutlineSource::Id outline_source_);
    PolygonZWithAttributes(OGRGeometryPtr geometry, const std::string &id_,
                           const OutlineSource::Id outline_source_);

    std::string get_id() const override;
    OutlineSource::Id get_outline_source() const override;
    std::unique_ptr<Geometry> clone() const override;
};

struct MultiPolygonZ : public virtual Geometry {
    OGRMultiPolygonPtr multi_polygon;

    MultiPolygonZ() = default;
    MultiPolygonZ(OGRMultiPolygonPtr multi_polygon_);
    MultiPolygonZ(OGRGeometryPtr geometry);
    MultiPolygonZ(const PolygonZ &polygon);
    MultiPolygonZ(const std::vector<PolygonZ> &polygons_);
    MultiPolygonZ(const MultiPolygonZ &other);

    OGRGeometryPtr get_geom() const override;
    OGREnvelopePtr bounding_box() const override;
    std::unique_ptr<Geometry> clone() const override;
};

struct MultiPolygonZWithAttributes : MultiPolygonZ, GeometryWithAttributes {
    std::string id;
    OutlineSource::Id outline_source;

    MultiPolygonZWithAttributes() = default;
    MultiPolygonZWithAttributes(MultiPolygonZ multi_polygon_,
                                const std::string &id_,
                                const OutlineSource::Id outline_source_);
    MultiPolygonZWithAttributes(OGRMultiPolygonPtr multi_polygon_,
                                const std::string &id_,
                                const OutlineSource::Id outline_source_);
    MultiPolygonZWithAttributes(OGRGeometryPtr geometry, const std::string &id_,
                                const OutlineSource::Id outline_source_);
    MultiPolygonZWithAttributes(const PolygonZ &polygon, const std::string &id_,
                                const OutlineSource::Id outline_source_);
    MultiPolygonZWithAttributes(const std::vector<PolygonZ> &polygons_,
                                const std::string &id_,
                                const OutlineSource::Id outline_source_);
    MultiPolygonZWithAttributes(
        const PolygonZWithAttributes &polygon_with_attributes);

    PolygonZWithAttributes get_polygon_with_attributes(int index) const;
    std::vector<PolygonZWithAttributes> get_polygons_with_attributes() const;
    OGREnvelopePtr bounding_box() const override;
    std::vector<OGREnvelopePtr> bounding_boxes() const;

    std::string get_id() const override;
    OutlineSource::Id get_outline_source() const override;
    std::unique_ptr<Geometry> clone() const override;
};

struct MultiLineStringZ : public virtual Geometry {
    OGRMultiLineStringPtr multi_line_string;

    MultiLineStringZ() = default;
    MultiLineStringZ(OGRMultiLineStringPtr multi_line_string_);
    MultiLineStringZ(OGRGeometryPtr geometry);
    MultiLineStringZ(const MultiLineStringZ &other);

    void add_line(const Segment_3 &segment);
    OGRGeometryPtr get_geom() const override;
    OGREnvelopePtr bounding_box() const override;
    std::unique_ptr<Geometry> clone() const override;
};

struct MultiLineStringZWithAttributes : MultiLineStringZ,
                                        GeometryWithAttributes {
    std::string id;
    OutlineSource::Id outline_source;

    MultiLineStringZWithAttributes() = default;
    MultiLineStringZWithAttributes(MultiLineStringZ multi_line_string_,
                                   const std::string &id_,
                                   const OutlineSource::Id outline_source_);
    MultiLineStringZWithAttributes(OGRMultiLineStringPtr multi_line_string_,
                                   const std::string &id_,
                                   const OutlineSource::Id outline_source_);
    MultiLineStringZWithAttributes(OGRGeometryPtr geometry,
                                   const std::string &id_,
                                   const OutlineSource::Id outline_source_);

    std::string get_id() const override;
    OutlineSource::Id get_outline_source() const override;
    std::unique_ptr<Geometry> clone() const override;
};

inline int64_t building_id_to_int64(const std::string &id) {
    try {
        return std::stoll(id.substr(8, 16));
    } catch (const std::exception &e) {
        throw std::runtime_error("Failed to convert ID to int64: " + id);
    }
}

arrow::Result<OGRMultiLineStringPtr>
parse_wkb_multilinestringz(const std::vector<uint8_t> &wkb);
arrow::Result<OGRMultiPolygonPtr>
parse_wkb_multipolygonz(const std::vector<uint8_t> &wkb);