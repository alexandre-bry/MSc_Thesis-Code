#include "writer.hpp"

#include <string>
#include <sys/types.h>

#include <arrow/api.h>
#include <arrow/array/builder_binary.h>
#include <arrow/array/builder_nested.h>
#include <arrow/io/file.h>
#include <arrow/scalar.h>
#include <arrow/status.h>
#include <arrow/type.h>
#include <arrow/type_fwd.h>
#include <parquet/arrow/reader.h>
#include <parquet/arrow/writer.h>

#include <ogr_geometry.h>
#include <ogr_spatialref.h>

#include "json/json.hpp"

using json = nlohmann::json;

std::string ProjjsonFromEPSG(const std::string &epsg) {
    OGRSpatialReference srs;
    if (srs.SetFromUserInput(epsg.c_str()) != OGRERR_NONE) {
        throw std::runtime_error("Failed to parse CRS: " + epsg);
    }
    char *projjson_raw = nullptr;
    if (srs.exportToPROJJSON(&projjson_raw, nullptr) != OGRERR_NONE) {
        throw std::runtime_error("Failed to export PROJJSON");
    }
    std::string projjson(projjson_raw);
    CPLFree(projjson_raw);
    return projjson;
}

char *buildGeoMetaData(std::string crs_epsg, std::string geometry_type,
                       std::string primary_column) {

    // Get PROJJSON from EPSG
    std::string projjson = ProjjsonFromEPSG(crs_epsg);

    // Build metadata object
    json metadata;
    metadata["version"] = "1.1.0";
    metadata["primary_column"] = primary_column;

    metadata["columns"] = json::object();
    json &col = metadata["columns"][primary_column];
    col["encoding"] = "WKB";
    col["geometry_types"] = json::array({geometry_type});
    col["crs"] = json::parse(projjson); // PROJJSON is already valid JSON
    col["covering"] = json::object();
    col["covering"]["bbox"]["xmin"] = json::array({"bbox", "xmin"});
    col["covering"]["bbox"]["ymin"] = json::array({"bbox", "ymin"});
    col["covering"]["bbox"]["xmax"] = json::array({"bbox", "xmax"});
    col["covering"]["bbox"]["ymax"] = json::array({"bbox", "ymax"});

    // Serialize to string and allocate char*
    std::string json_str = metadata.dump(2); // Pretty-print with 2 spaces
    char *result = (char *)CPLMalloc(json_str.size() + 1);
    std::strcpy(result, json_str.c_str());

    return result;
}
