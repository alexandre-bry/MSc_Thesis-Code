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

struct OpenFileException : public std::exception {
    mutable std::string message;

    OpenFileException(const std::string &filename) {
        this->message = "Error opening the file " + filename;
    }

    const char *what() const throw() { return message.c_str(); }
};

struct ObjTriangle {
    std::string mesh_name;
    uint material;
    Triangle_3 geometry;
};

std::ostream &operator<<(std::ostream &os, const ObjTriangle &obj_triangle);

std::vector<ObjTriangle> read_obj(const std::string &filename);
void write_obj(const std::string &filename,
               std::vector<ObjTriangle> obj_triangles);
