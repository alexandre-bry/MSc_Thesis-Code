#pragma once

#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "geometries.h"
#include "helper.h"

#include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>

struct OpenFileException : public std::exception {
    mutable std::string message;

    OpenFileException(const std::string &filename) {
        this->message = "Error opening the file " + filename;
    }

    const char *what() const throw() { return message.c_str(); }
};

struct WrongInputException : public std::exception {
    mutable std::string message;

    WrongInputException(const std::string &message) { this->message = message; }

    const char *what() const throw() { return message.c_str(); }
};

enum Material { Floor, Façade, Roof };

const std::unordered_map<uint, Material> IntToMaterial = {
    {0, Floor},
    {1, Roof},
    {2, Façade},

};
const std::unordered_map<Material, std::string> MaterialToString = {
    {Floor, "Floor"},
    {Façade, "Façade"},
    {Roof, "Roof"},
};

struct ObjTriangle {
    std::string mesh_name;
    Material material;
    Triangle_3 geometry;
};

std::ostream &operator<<(std::ostream &os, const ObjTriangle &obj_triangle);
std::ostream &operator<<(std::ostream &os, const Material &material);

std::vector<ObjTriangle> read_obj(const std::string &filename);
// std::unordered_map<Material, Polyhedron_3>
// polys_from_obj_triangles(std::vector<ObjTriangle> obj_triangles);
void write_obj(const std::string &filename,
               std::vector<ObjTriangle> obj_triangles);
void split_obj(const std::string &in_filename, const std::string &out_filename);