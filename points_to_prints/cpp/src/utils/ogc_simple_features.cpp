#include "ogc_simple_features.hpp"

#include <utility>

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

/* -------------------------------------------------------------------------- */
/*                                  PolygonZ                                  */
/* -------------------------------------------------------------------------- */

PolygonZ::PolygonZ(OGRPolygonPtr polygon_) : polygon(std::move(polygon_)) {}

PolygonZ::PolygonZ(OGRGeometryPtr geometry) {
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
}

PolygonZ::PolygonZ(const std::vector<Point_3> &points, bool first_is_repeated) {
    OGRPolygon *polygon = new OGRPolygon();
    OGRLinearRing *ring = new OGRLinearRing();
    for (const auto &p : points) {
        ring->addPoint(p.x(), p.y(), p.z());
    }
    if (!first_is_repeated) {
        ring->addPoint(points[0].x(), points[0].y(), points[0].z());
    }
    polygon->addRing(ring);
    this->polygon.reset(polygon);
}

PolygonZ::PolygonZ(const std::vector<std::vector<Point_3>> &rings,
                   bool first_is_repeated) {
    OGRPolygon *polygon = new OGRPolygon();
    for (const auto &ring_points : rings) {
        OGRLinearRing *ring = new OGRLinearRing();
        for (const auto &p : ring_points) {
            ring->addPoint(p.x(), p.y(), p.z());
        }
        if (!first_is_repeated) {
            ring->addPoint(ring_points[0].x(), ring_points[0].y(),
                           ring_points[0].z());
        }
        polygon->addRing(ring);
    }
    this->polygon.reset(polygon);
}

PolygonZ::PolygonZ(const PolygonZ &other) {
    OGRPolygon *polygon_copy =
        dynamic_cast<OGRPolygon *>(other.polygon->clone());
    if (!polygon_copy) {
        throw std::runtime_error("Failed to clone polygon");
    }
    polygon.reset(polygon_copy);
}

OGRGeometryPtr PolygonZ::get_geom() const {
    OGRPolygon *polygon_copy = dynamic_cast<OGRPolygon *>(polygon->clone());
    if (!polygon_copy) {
        throw std::runtime_error("Failed to clone polygon");
    }
    return OGRGeometryPtr(polygon_copy);
}

OGREnvelopePtr PolygonZ::bounding_box() const {
    OGREnvelope env;
    polygon->getEnvelope(&env);
    return std::make_unique<OGREnvelope>(env);
}

std::unique_ptr<Geometry> PolygonZ::clone() const {
    return std::make_unique<PolygonZ>(*this);
}

/* -------------------------------------------------------------------------- */
/*                           PolygonZWithAttributes                           */
/* -------------------------------------------------------------------------- */

PolygonZWithAttributes::PolygonZWithAttributes(
    OGRPolygonPtr geometry, const std::string &id_,
    const OutlineSource::Id outline_source_)
    : PolygonZ(std::move(geometry)), id(id_), outline_source(outline_source_) {}

PolygonZWithAttributes::PolygonZWithAttributes(
    OGRGeometryPtr geometry, const std::string &id_,
    const OutlineSource::Id outline_source_)
    : PolygonZ(std::move(geometry)), id(id_), outline_source(outline_source_) {}

std::string PolygonZWithAttributes::get_id() const { return id; }

OutlineSource::Id PolygonZWithAttributes::get_outline_source() const {
    return outline_source;
}

std::unique_ptr<Geometry> PolygonZWithAttributes::clone() const {
    return std::make_unique<PolygonZWithAttributes>(*this);
}

/* -------------------------------------------------------------------------- */
/*                                MultiPolygonZ                               */
/* -------------------------------------------------------------------------- */

MultiPolygonZ::MultiPolygonZ(OGRMultiPolygonPtr multi_polygon_)
    : multi_polygon(std::move(multi_polygon_)) {}

MultiPolygonZ::MultiPolygonZ(OGRGeometryPtr geometry) {
    if (geometry->IsSimple()) {
        throw std::runtime_error("Geometry is simple");
    }
    if (geometry->getCoordinateDimension() != 3) {
        throw std::runtime_error("Geometry is not 3D");
    }
    if (!geometry->IsValid()) {
        throw std::runtime_error("Geometry is not valid");
    }

    multi_polygon.reset(dynamic_cast<OGRMultiPolygon *>(geometry.release()));
    if (!multi_polygon) {
        throw std::runtime_error("Failed to cast geometry to MultiPolygon");
    }
}

MultiPolygonZ::MultiPolygonZ(const PolygonZ &polygon) {
    multi_polygon.reset(new OGRMultiPolygon());
    if (multi_polygon->addGeometry(polygon.polygon.get()) != OGRERR_NONE) {
        throw std::runtime_error("Failed to add polygon to multipolygon");
    }
}

MultiPolygonZ::MultiPolygonZ(const std::vector<PolygonZ> &polygons_) {
    multi_polygon.reset(new OGRMultiPolygon());
    for (const auto &polygon : polygons_) {
        if (multi_polygon->addGeometry(polygon.polygon.get()) != OGRERR_NONE) {
            throw std::runtime_error("Failed to add polygon to multipolygon");
        }
    }
}

MultiPolygonZ::MultiPolygonZ(const MultiPolygonZ &other) {
    OGRMultiPolygon *multi_polygon_copy =
        dynamic_cast<OGRMultiPolygon *>(other.multi_polygon->clone());
    if (!multi_polygon_copy) {
        throw std::runtime_error("Failed to clone multipolygon");
    }
    multi_polygon.reset(multi_polygon_copy);
}

OGRGeometryPtr MultiPolygonZ::get_geom() const {
    OGRMultiPolygon *multi_polygon_copy =
        dynamic_cast<OGRMultiPolygon *>(multi_polygon->clone());
    if (!multi_polygon_copy) {
        throw std::runtime_error("Failed to clone multipolygon");
    }
    return OGRGeometryPtr(multi_polygon_copy);
}

OGREnvelopePtr MultiPolygonZ::bounding_box() const {
    OGREnvelope env;
    multi_polygon->getEnvelope(&env);
    return std::make_unique<OGREnvelope>(env);
}

std::unique_ptr<Geometry> MultiPolygonZ::clone() const {
    return std::make_unique<MultiPolygonZ>(*this);
}

/* -------------------------------------------------------------------------- */
/*                         MultiPolygonZWithAttributes                        */
/* -------------------------------------------------------------------------- */

MultiPolygonZWithAttributes::MultiPolygonZWithAttributes(
    MultiPolygonZ multi_polygon_, const std::string &id_,
    const OutlineSource::Id outline_source_)
    : MultiPolygonZ(std::move(multi_polygon_)), id(id_),
      outline_source(outline_source_) {}

MultiPolygonZWithAttributes::MultiPolygonZWithAttributes(
    OGRMultiPolygonPtr multi_polygon_, const std::string &id_,
    const OutlineSource::Id outline_source_)
    : MultiPolygonZ(std::move(multi_polygon_)), id(id_),
      outline_source(outline_source_) {}

MultiPolygonZWithAttributes::MultiPolygonZWithAttributes(
    OGRGeometryPtr geometry, const std::string &id_,
    const OutlineSource::Id outline_source_)
    : MultiPolygonZ(std::move(geometry)), id(id_),
      outline_source(outline_source_) {}

MultiPolygonZWithAttributes::MultiPolygonZWithAttributes(
    const PolygonZ &polygon, const std::string &id_,
    const OutlineSource::Id outline_source_)
    : MultiPolygonZ(polygon), id(id_), outline_source(outline_source_) {}

MultiPolygonZWithAttributes::MultiPolygonZWithAttributes(
    const std::vector<PolygonZ> &polygons_, const std::string &id_,
    const OutlineSource::Id outline_source_)
    : MultiPolygonZ(polygons_), id(id_), outline_source(outline_source_) {}

MultiPolygonZWithAttributes::MultiPolygonZWithAttributes(
    const PolygonZWithAttributes &polygon_with_attributes)
    : MultiPolygonZ(polygon_with_attributes), id(polygon_with_attributes.id),
      outline_source(polygon_with_attributes.outline_source) {}

PolygonZWithAttributes
MultiPolygonZWithAttributes::get_polygon_with_attributes(int index) const {
    OGRPolygon *polygon =
        dynamic_cast<OGRPolygon *>(multi_polygon->getGeometryRef(index));
    if (!polygon) {
        throw std::runtime_error("Failed to cast geometry to Polygon");
    }
    OGRPolygon *polygon_copy = dynamic_cast<OGRPolygon *>(polygon->clone());
    if (!polygon_copy) {
        throw std::runtime_error("Failed to clone polygon");
    }
    return PolygonZWithAttributes(OGRPolygonPtr(polygon_copy), id,
                                  outline_source);
}

std::vector<PolygonZWithAttributes>
MultiPolygonZWithAttributes::get_polygons_with_attributes() const {
    std::vector<PolygonZWithAttributes> polygons;
    for (int i = 0; i < multi_polygon->getNumGeometries(); i++) {
        polygons.push_back(get_polygon_with_attributes(i));
    }
    return polygons;
}

OGREnvelopePtr MultiPolygonZWithAttributes::bounding_box() const {
    OGREnvelope env;
    multi_polygon->getEnvelope(&env);
    return std::make_unique<OGREnvelope>(env);
}

std::vector<OGREnvelopePtr>
MultiPolygonZWithAttributes::bounding_boxes() const {
    std::vector<OGREnvelopePtr> boxes(multi_polygon->getNumGeometries());
    for (int i = 0; i < multi_polygon->getNumGeometries(); i++) {
        PolygonZWithAttributes polygon = get_polygon_with_attributes(i);
        boxes[i] = polygon.bounding_box();
    }
    return boxes;
}

std::string MultiPolygonZWithAttributes::get_id() const { return id; }

OutlineSource::Id MultiPolygonZWithAttributes::get_outline_source() const {
    return outline_source;
}

std::unique_ptr<Geometry> MultiPolygonZWithAttributes::clone() const {
    return std::make_unique<MultiPolygonZWithAttributes>(*this);
}

/* -------------------------------------------------------------------------- */
/*                              MultiLineStringZ                              */
/* -------------------------------------------------------------------------- */

MultiLineStringZ::MultiLineStringZ(OGRMultiLineStringPtr multi_line_string_)
    : multi_line_string(std::move(multi_line_string_)) {}

MultiLineStringZ::MultiLineStringZ(OGRGeometryPtr geometry) {
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
        throw std::runtime_error("Failed to cast geometry to MultiLineString");
    }
}

MultiLineStringZ::MultiLineStringZ(const MultiLineStringZ &other) {
    OGRMultiLineString *multi_line_string_copy =
        dynamic_cast<OGRMultiLineString *>(other.multi_line_string->clone());
    if (!multi_line_string_copy) {
        throw std::runtime_error("Failed to clone multilinestring");
    }
    multi_line_string.reset(multi_line_string_copy);
}

void MultiLineStringZ::add_line(const Segment_3 &segment) {
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

OGRGeometryPtr MultiLineStringZ::get_geom() const {
    OGRMultiLineString *multi_line_string_copy =
        dynamic_cast<OGRMultiLineString *>(multi_line_string->clone());
    if (!multi_line_string_copy) {
        throw std::runtime_error("Failed to clone multilinestring");
    }
    return OGRGeometryPtr(multi_line_string_copy);
}

OGREnvelopePtr MultiLineStringZ::bounding_box() const {
    OGREnvelope env;
    multi_line_string->getEnvelope(&env);
    return std::make_unique<OGREnvelope>(env);
}

std::unique_ptr<Geometry> MultiLineStringZ::clone() const {
    return std::make_unique<MultiLineStringZ>(*this);
}

/* -------------------------------------------------------------------------- */
/*                       MultiLineStringZWithAttributes                       */
/* -------------------------------------------------------------------------- */

MultiLineStringZWithAttributes::MultiLineStringZWithAttributes(
    MultiLineStringZ multi_line_string_, const std::string &id_,
    const OutlineSource::Id outline_source_)
    : MultiLineStringZ(std::move(multi_line_string_)), id(id_),
      outline_source(outline_source_) {}

MultiLineStringZWithAttributes::MultiLineStringZWithAttributes(
    OGRMultiLineStringPtr multi_line_string_, const std::string &id_,
    const OutlineSource::Id outline_source_)
    : MultiLineStringZ(std::move(multi_line_string_)), id(id_),
      outline_source(outline_source_) {}

MultiLineStringZWithAttributes::MultiLineStringZWithAttributes(
    OGRGeometryPtr geometry, const std::string &id_,
    const OutlineSource::Id outline_source_)
    : MultiLineStringZ(std::move(geometry)), id(id_),
      outline_source(outline_source_) {}

std::string MultiLineStringZWithAttributes::get_id() const { return id; }

OutlineSource::Id MultiLineStringZWithAttributes::get_outline_source() const {
    return outline_source;
}

std::unique_ptr<Geometry> MultiLineStringZWithAttributes::clone() const {
    return std::make_unique<MultiLineStringZWithAttributes>(*this);
}

arrow::Result<OGRMultiLineStringPtr>
parse_wkb_multilinestringz(const std::vector<uint8_t> &wkb) {
    if (wkb.empty()) {
        return arrow::Status::Invalid("Geometry WKB is empty");
    }

    OGRGeometry *geom_raw = nullptr;
    OGRErr err = OGRGeometryFactory::createFromWkb(
        wkb.data(), getLAMB93(), &geom_raw, static_cast<int>(wkb.size()));
    if (err != OGRERR_NONE || geom_raw == nullptr) {
        return arrow::Status::Invalid("Failed to parse geometry WKB");
    }

    OGRGeometryPtr geom(geom_raw);
    if (wkbFlatten(geom->getGeometryType()) != wkbMultiLineString) {
        return arrow::Status::TypeError("Geometry is not a MultiLineString");
    }

    OGRMultiLineString *multi_line_string_raw =
        geom.release()->toMultiLineString();
    if (multi_line_string_raw == nullptr) {
        return arrow::Status::Invalid(
            "Failed to cast geometry to OGRMultiLineString");
    }

    // Check if the geometry is 3D or empty (in which case we consider it as 3D)
    if (!multi_line_string_raw->IsEmpty() &&
        multi_line_string_raw->getCoordinateDimension() != 3) {
        return arrow::Status::TypeError(
            "Geometry is MultiLineString but not 3D (MultiLineStringZ)");
    }

    return OGRMultiLineStringPtr(multi_line_string_raw);
}

arrow::Result<OGRMultiPolygonPtr>
parse_wkb_multipolygonz(const std::vector<uint8_t> &wkb) {
    if (wkb.empty()) {
        return arrow::Status::Invalid("Geometry WKB is empty");
    }

    OGRGeometry *geom_raw = nullptr;
    OGRErr err = OGRGeometryFactory::createFromWkb(
        wkb.data(), getLAMB93(), &geom_raw, static_cast<int>(wkb.size()));
    if (err != OGRERR_NONE || geom_raw == nullptr) {
        return arrow::Status::Invalid("Failed to parse geometry WKB");
    }

    OGRGeometryPtr geom(geom_raw);
    if (wkbFlatten(geom->getGeometryType()) != wkbMultiPolygon) {
        return arrow::Status::TypeError("Geometry is not a MultiPolygon");
    }

    OGRMultiPolygon *multi_polygon_raw = geom.release()->toMultiPolygon();
    if (multi_polygon_raw == nullptr) {
        return arrow::Status::Invalid(
            "Failed to cast geometry to OGRMultiPolygon");
    }

    if (multi_polygon_raw->getCoordinateDimension() != 3) {
        return arrow::Status::TypeError(
            "Geometry is MultiPolygon but not 3D (MultiPolygonZ)");
    }

    return OGRMultiPolygonPtr(multi_polygon_raw);
}