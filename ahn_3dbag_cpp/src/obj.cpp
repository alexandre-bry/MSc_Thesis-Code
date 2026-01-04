#include "obj.h"

std::ostream &operator<<(std::ostream &os, const ObjTriangle &obj_triangle) {
    os << "(" << obj_triangle.mesh_name << ", " << obj_triangle.material << ", "
       << obj_triangle.geometry << ")";
    return os;
}

/// Reads the given OBJ with its objects and materials.
std::vector<ObjTriangle> read_obj(const std::string &filename) {
    std::cout << "Reading " << filename << "..." << std::endl;
    std::ifstream objfile(filename);
    if (!objfile.is_open()) {
        throw OpenFileException(filename);
    }

    std::vector<Point_3> vertices;
    std::vector<ObjTriangle> objects;
    std::string line;
    uint material;
    std::string name;
    while (std::getline(objfile, line)) {
        // Skip comments
        if (line.rfind("#", 0) == 0) {
            continue;
        }

        // Skip material file
        if (line.rfind("mtllib", 0) == 0) {
            continue;
        }

        if (line.rfind("o", 0) == 0) {
            // Object name
            std::istringstream iss(line);
            std::string prefix;
            if (!(iss >> prefix >> name)) {
                std::cerr << "Error reading an object name!" << std::endl;
                break;
            }
        } else if (line.rfind("usemtl", 0) == 0) {
            // Face material
            std::istringstream iss(line);
            std::string prefix;
            if (!(iss >> prefix >> material)) {
                std::cerr << "Error reading a face material!" << std::endl;
                break;
            }
        } else if (line.rfind("v", 0) == 0) {
            // Vertex
            std::istringstream iss(line);
            std::string prefix;
            double x, y, z;
            if (!(iss >> prefix >> x >> y >> z)) {
                std::cerr << "Error reading a vertex!" << std::endl;
                break;
            }
            vertices.push_back({x, y, z});
        }

        else if (line.rfind("f", 0) == 0) {
            // Face
            std::istringstream iss(line);
            std::string prefix;
            uint v0, v1, v2;
            if (!(iss >> prefix >> v0 >> v1 >> v2)) {
                std::cerr << "Error reading a face!" << std::endl;
                break;
            }
            // OBJ starts at 1
            v0 -= 1;
            v1 -= 1;
            v2 -= 1;
            const Triangle_3 triangle(vertices.at(v0), vertices.at(v1),
                                      vertices.at(v2));
            objects.push_back({name, material, triangle});
        }
    }

    std::cout << "Finished reading!" << std::endl;
    return objects;
}

/// Writes the given triangles to OBJ.
///
/// Writes very non optimally because identical vertices are repeated.
void write_obj(const std::string &filename,
               std::vector<ObjTriangle> obj_triangles) {
    std::cout << "Writing " << filename << "..." << std::endl;
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        throw OpenFileException(filename);
    }

    std::unordered_map<std::string,
                       std::unordered_map<uint, std::vector<Triangle_3>>>
        mappings;

    for (const auto &obj_triangle : obj_triangles) {
        if (mappings.find(obj_triangle.mesh_name) == mappings.end()) {
            mappings[obj_triangle.mesh_name] = {};
        }
        if (mappings[obj_triangle.mesh_name].find(obj_triangle.material) ==
            mappings[obj_triangle.mesh_name].end()) {
            mappings[obj_triangle.mesh_name][obj_triangle.material] = {};
        }

        mappings[obj_triangle.mesh_name][obj_triangle.material].push_back(
            obj_triangle.geometry);
    }

    outfile << "mtllib output.obj.mtl" << std::endl;

    // Write vertices
    for (const auto &[object_name, material_triangle] : mappings) {
        for (const auto &[material, triangles] : material_triangle) {
            for (const auto &triangle : triangles) {
                for (uint i = 0; i < 3; i++) {
                    outfile << "v " << std::setprecision(15)
                            << triangle.vertex(i).x() << " "
                            << triangle.vertex(i).y() << " "
                            << triangle.vertex(i).z() << std::endl;
                }
            }
        }
    }

    // Write triangles
    uint count = 1;
    for (const auto &[object_name, material_triangle] : mappings) {
        outfile << "o " << object_name << std::endl;
        for (const auto &[material, triangles] : material_triangle) {
            outfile << "usemtl " << material << std::endl;
            for (const auto &triangle : triangles) {
                outfile << "f " << count++ << " " << count++ << " " << count++
                        << std::endl;
            }
        }
    }

    std::cout << "Finished writing!" << std::endl;
}