
#include <CGAL/Named_function_parameters.h>
#include <exception>
#include <iostream>
#include <vector>

#include "extension.h"
#include "geometries.h"
#include "obj.h"
#include <CGAL/IO/OBJ.h>

int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <command> ...\n";
        return EXIT_FAILURE;
    }

    try {
        if (std::string("split") == argv[1]) {
            if (argc != 4) {
                std::cerr << "Usage: " << argv[0]
                          << " split <input.obj> <output.obj>\n";
                return EXIT_FAILURE;
            }
            split_obj(argv[2], argv[3]);
        } else if (std::string("extend") == argv[1]) {
            std::vector<Point_3> points;
            std::vector<std::vector<size_t>> triangles;
            CGAL::IO::read_OBJ(argv[1], points, triangles);
            CGAL::IO::write_OBJ(argv[2], points, triangles,
                                CGAL::parameters::stream_precision(11));
            // auto meshes = read_obj(argv[1]);
            // std::vector<ObjTriangle> updated_meshes;
            // for (const auto mesh : meshes) {
            //     if (mesh.material == 1) {
            //         const auto new_triangles = extend_triangle(mesh.geometry,
            //         1); for (const auto &new_triangle : new_triangles) {
            //             updated_meshes.push_back(
            //                 {mesh.mesh_name, mesh.material, new_triangle});
            //         }
            //     }
            // }
            // write_obj(argv[2], meshes);
        } else {
            std::cerr << "Unknown command: " << argv[1] << "\n"
                      << "Usage: " << argv[0] << " <command> ...\n";
            return EXIT_FAILURE;
        }
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << "\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}