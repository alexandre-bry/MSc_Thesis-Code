#pragma once

#include <cmath>
#include <cstdint>
#include <string>

#include <pdal/Dimension.hpp>
#include <pdal/PointView.hpp>
#include <pdal/pdal_types.hpp>

namespace LASclassification {
enum class Value : uint8_t {
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
    Overlap = 12,
    WireGuard = 13,
    WireConductor = 14,
    TransmissionTower = 15,
    WireConnector = 16,
    BridgeDeck = 17,
    HighNoise = 18,
    PermanentOverground = 64,    // Specific to LiDAR HD
    VirtualPoints = 66,          // Specific to LiDAR HD
    MiscellaneousBuildings = 67, // Specific to LiDAR HD
};
inline std::string name(Value value) {
    switch (value) {
    case Value::Unclassified:
        return "Unclassified";
    case Value::Unassigned:
        return "Unassigned";
    case Value::Ground:
        return "Ground";
    case Value::LowVegetation:
        return "Low Vegetation";
    case Value::MediumVegetation:
        return "Medium Vegetation";
    case Value::HighVegetation:
        return "High Vegetation";
    case Value::Building:
        return "Building";
    case Value::LowPoint:
        return "Low Point";
    case Value::ModelKeyPoint:
        return "Model Key Point";
    case Value::Water:
        return "Water";
    case Value::Rail:
        return "Rail";
    case Value::RoadSurface:
        return "Road Surface";
    case Value::Overlap:
        return "Overlap";
    case Value::WireGuard:
        return "Wire Guard";
    case Value::WireConductor:
        return "Wire Conductor";
    case Value::TransmissionTower:
        return "Transmission Tower";
    case Value::WireConnector:
        return "Wire Connector";
    case Value::BridgeDeck:
        return "Bridge Deck";
    case Value::HighNoise:
        return "High Noise";
    case Value::PermanentOverground:
        return "Permanent Overground (LiDAR HD)";
    case Value::VirtualPoints:
        return "Virtual Points (LiDAR HD)";
    case Value::MiscellaneousBuildings:
        return "Miscellaneous Buildings (LiDAR HD)";
    default:
        throw std::runtime_error("Unknown LAS classification value: " +
                                 std::to_string(static_cast<uint8_t>(value)));
    }
}
} // namespace LASclassification

namespace CustomDimensions {
enum class Id {
    ReturnNumberComputed,
    NumberOfReturnsComputed,
    InwardVectorX,
    InwardVectorY,
    InwardVectorZ,
    MaxVerticalDiff,
    MinVerticalDiff,
    IsRoofEdge,
    IsFootEdge,
    IsFacade,
    IsGenerated,
    ScannerPositionX,
    ScannerPositionY,
    ScannerPositionZ,
    CorrespondingBuildingId
};
inline std::string name(Id id) {
    switch (id) {
    case Id::ReturnNumberComputed:
        return "ReturnNumberComputed";
    case Id::NumberOfReturnsComputed:
        return "NumberOfReturnsComputed";
    case Id::InwardVectorX:
        return "InwardVectorX";
    case Id::InwardVectorY:
        return "InwardVectorY";
    case Id::InwardVectorZ:
        return "InwardVectorZ";
    case Id::MaxVerticalDiff:
        return "MaxVerticalDiff";
    case Id::MinVerticalDiff:
        return "MinVerticalDiff";
    case Id::IsRoofEdge:
        return "IsRoofEdge";
    case Id::IsFootEdge:
        return "IsFootEdge";
    case Id::IsFacade:
        return "IsFacade";
    case Id::IsGenerated:
        return "IsGenerated";
    case Id::ScannerPositionX:
        return "ScannerPositionX";
    case Id::ScannerPositionY:
        return "ScannerPositionY";
    case Id::ScannerPositionZ:
        return "ScannerPositionZ";
    case Id::CorrespondingBuildingId:
        return "CorrespondingBuildingId";
    default:
        throw std::runtime_error("Unknown custom dimension ID");
    }
}

inline pdal::Dimension::Type type(Id id) {
    switch (id) {
    case Id::ReturnNumberComputed:
        return pdal::Dimension::Type::Unsigned8;
    case Id::NumberOfReturnsComputed:
        return pdal::Dimension::Type::Unsigned8;
    case Id::InwardVectorX:
        return pdal::Dimension::Type::Double;
    case Id::InwardVectorY:
        return pdal::Dimension::Type::Double;
    case Id::InwardVectorZ:
        return pdal::Dimension::Type::Double;
    case Id::MaxVerticalDiff:
        return pdal::Dimension::Type::Double;
    case Id::MinVerticalDiff:
        return pdal::Dimension::Type::Double;
    case Id::IsRoofEdge:
        return pdal::Dimension::Type::Unsigned8;
    case Id::IsFootEdge:
        return pdal::Dimension::Type::Unsigned8;
    case Id::IsGenerated:
        return pdal::Dimension::Type::Unsigned8;
    case Id::IsFacade:
        return pdal::Dimension::Type::Unsigned8;
    case Id::ScannerPositionX:
        return pdal::Dimension::Type::Double;
    case Id::ScannerPositionY:
        return pdal::Dimension::Type::Double;
    case Id::ScannerPositionZ:
        return pdal::Dimension::Type::Double;
    case Id::CorrespondingBuildingId:
        return pdal::Dimension::Type::Unsigned64;
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
