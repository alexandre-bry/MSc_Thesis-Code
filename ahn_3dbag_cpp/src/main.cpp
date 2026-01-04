
#include <exception>
#include <iostream>
#include <vector>

#include "minkowski.h"
#include "obj.h"

int main(int argc, char *argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <input.obj> <output.obj>\n";
        return EXIT_FAILURE;
    }

    try {
        auto meshes = read_obj(argv[1]);
        std::vector<ObjTriangle> updated_meshes;
        for (const auto mesh : meshes) {
            if (mesh.material == 1) {
                const auto new_triangles = extend_triangle(mesh.geometry, 1);
                for (const auto &new_triangle : new_triangles) {
                    updated_meshes.push_back(
                        {mesh.mesh_name, mesh.material, new_triangle});
                }
            }
        }
        write_obj(argv[2], meshes);
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << "\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}