#pragma once
#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include <memory>
#include "Vectors.h"
#include "config.h"
#include "Object.h"


class Plane: public Object {
public:
    Vector3f normal;
    double offset;
    
    Plane(): Object(PLANE) {}
    Plane(Vector3f _normal, double _offset): Object(PLANE), normal(_normal), offset(_offset) {}
    ~Plane() {}

    Intersection intersect(const Ray &ray, unsigned short *Xi) const override {
        Intersection ret;
        double prod = dot(normal, ray.d);
        // std::cout << "prod = " << prod << "t = " << (offset - dot(normal, ray.o)) / prod << std::endl;
        if (abs(prod) < EPS || (ret.t = (offset - dot(normal, ray.o)) / prod) < EPS) return Intersection();
        // std::cout << "intersect with plane!" << std::endl;
        ret.type = prod < 0 ? INTO: OUTFROM;
        ret.poc = ray.o + ret.t * ray.d;
        // ret.normal = prod > 0 ? -1 * normal : normal;
        ret.normal = normal;
        ret.material = material;
        return ret;
    }

    friend std::istream &operator >>(std::istream &fin, Plane &plane);
};

std::istream &operator >>(std::istream &fin, Plane &plane) {
    std::string s;
    bool _normal = false, _offset = false; // the input must contain these parameters
    while (fin >> s) {
        if (s.length() == 0 || s[0] == '#') {
            fin.ignore(256, '\n');
            continue;
        }
        if (s == "end") break;
        else if (s == "material") {
            plane.material.reset(new Material());
            fin >> *(plane.material);
        }
        else if (s == "normal") fin >> plane.normal, _normal = true;
        else if (s == "offset") fin >> plane.offset, _offset = true;
        else {
            std::cerr << std::string("Error: Unrecognized plane parameter : ") + s << std::endl;
            exit(-1);
        }
    }
    if (!_normal || ! _offset) {
        std::string missing_str = !_normal ? "normal" : "offset";
        std::cerr << "Error: Plane parameter missing: " + missing_str << std::endl;
        exit(-1);
    }
    return fin;
}