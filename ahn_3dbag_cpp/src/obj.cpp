#include "obj.h"
#include "geometries.h"
#include <cstddef>
#include <unordered_map>
#include <vector>

std::ostream &operator<<(std::ostream &os, const ObjTriangle &obj_triangle) {
    os << "(" << obj_triangle.mesh_name << ", " << obj_triangle.material << ", "
       << obj_triangle.geometry << ")";
    return os;
}

std::ostream &operator<<(std::ostream &os, const Material &material) {
    switch (material) {
    case Floor:
        os << "Floor";
    case Roof:
        os << "Roof";
    case Façade:
        os << "Façade";
    default:
        throw WrongInputException("Incorrect material: " +
                                  std::to_string(material));
    }
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
    uint material_int;
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
            if (!(iss >> prefix >> material_int)) {
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
            Material material;
            switch (material_int) {
            case 0:
                material = Floor;
            case 1:
                material = Roof;
            case 2:
                material = Façade;
            default:
                throw WrongInputException("Incorrect material in " + filename);
            }
            objects.push_back({name, material, triangle});
        }
    }

    std::cout << "Finished reading!" << std::endl;
    return objects;
}

// std::unordered_map<Material, Polyhedron_3>
// polys_from_obj_triangles(std::vector<ObjTriangle> obj_triangles) {
//     std::unordered_map<Material, std::vector<Triangle_3>> triangles_sorted;

//     // Sort triangles per material
//     for (const auto &obj_triangle : obj_triangles) {
//         if (triangles_sorted.find(obj_triangle.material) ==
//             triangles_sorted.end()) {
//             triangles_sorted[obj_triangle.material] = {};
//         }

//         triangles_sorted[obj_triangle.material].push_back(
//             obj_triangle.geometry);
//     }

//     // Group triangles into one Polyhedron per material
//     for (const auto &[material, triangles] : triangles_sorted) {
//         const Polyhedron_3 poly;
//         for (const auto &triangle : triangles) {
//             // for (uint i = 0; i < 3; i++) {
//             //     outfile << "v " << std::setprecision(15)
//             //             << triangle.vertex(i).x() << " "
//             //             << triangle.vertex(i).y() << " "
//             //             << triangle.vertex(i).z() << std::endl;
//             // }
//         }
//     }
// }

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

void split_obj(const std::string &in_filename,
               const std::string &out_filename) {
    std::cout << "Splitting " << in_filename << "..." << std::endl;
    std::ifstream objfile(in_filename);
    if (!objfile.is_open()) {
        throw OpenFileException(in_filename);
    }

    std::vector<std::string> vertices;
    std::unordered_map<Material, std::vector<std::vector<size_t>>> mat_faces;

    // Read the file
    std::string line;
    uint material_int;
    while (std::getline(objfile, line)) {
        // Skip comments
        if (line.rfind("#", 0) == 0) {
            continue;
        }

        // Skip material file
        if (line.rfind("mtllib", 0) == 0) {
            continue;
        }

        if (line.rfind("usemtl", 0) == 0) {
            // Face material
            std::istringstream iss(line);
            std::string prefix;
            if (!(iss >> prefix >> material_int)) {
                std::cerr << "Error reading a face material!" << std::endl;
                break;
            }
        } else if (line.rfind("v", 0) == 0) {
            // Vertex
            vertices.push_back(line);
        } else if (line.rfind("f", 0) == 0) {
            // Face
            std::istringstream iss(line);
            std::string prefix;
            size_t v0, v1, v2;
            if (!(iss >> prefix >> v0 >> v1 >> v2)) {
                std::cerr << "Error reading a face!" << std::endl;
                break;
            }

            Material material = IntToMaterial.at(material_int);

            // Initialize key if not existing
            if (mat_faces.find(material) == mat_faces.end()) {
                mat_faces[material] = {};
            }
            // OBJ is 1-indexed
            mat_faces[material].push_back({v0 - 1, v1 - 1, v2 - 1});
        }
    }

    // Remap the vertices indices
    std::unordered_map<Material, std::unordered_map<size_t, size_t>>
        vertices_mapping;
    std::unordered_map<Material, std::vector<size_t>> vertices_new_order;
    for (const auto &[material, faces] : mat_faces) {
        vertices_mapping[material] = {};
        auto &mapping = vertices_mapping[material];
        auto &new_order = vertices_new_order[material];
        for (const auto &face : faces) {
            for (const auto &v_index : face) {
                if (mapping.find(v_index) == mapping.end()) {
                    // Vertex is not in yet
                    mapping[v_index] = mapping.size();
                    new_order.push_back(v_index);
                }
            }
        }
    }

    // Write the files
    for (const auto &[material, faces] : mat_faces) {
        // Prepare the output file
        auto outname = out_filename;
        outname.replace(outname.rfind(".obj"), 4,
                        std::string("-") + MaterialToString.at(material) +
                            ".obj");
        std::cout << outname << std::endl;
        std::ofstream outfile(outname);
        if (!outfile.is_open()) {
            throw OpenFileException(outname);
        }

        // Write the vertices
        for (const auto v_index_old : vertices_new_order[material]) {
            outfile << vertices[v_index_old] << std::endl;
        }

        // Write the faces
        for (const auto &face : faces) {
            outfile << "f";
            for (const auto v_index_old : face) {
                const auto v_index_new =
                    vertices_mapping[material][v_index_old];
                // OBJ is 1-indexed
                outfile << " " << v_index_new + 1;
            }
            outfile << std::endl;
        }
    }

    std::cout << "Finished splitting!" << std::endl;
}
