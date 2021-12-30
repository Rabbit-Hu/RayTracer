#pragma once
#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include <memory>
#include "Vectors.h"
#include "config.h"
#include "Canvas.h"
#include "Plane.h"


class Rectangle: public Object {
public:
    Vector3f pos = Vector3f(0, 0, 0);
    Vector3f up = Vector3f(0, 0, 1), right = Vector3f(0, 1, 0);
    double h, w;
    Plane plane;
    
    Rectangle(): Object(RECTANGLE) {}
    ~Rectangle() {}

    Intersection intersect(const Ray &ray, unsigned short *Xi) const override {
        Intersection ret = plane.intersect(ray, Xi);
        if (ret.type == MISS) return Intersection();
        ret.u = dot(ret.poc - pos, right) / w, ret.v = dot(ret.poc - pos, up) / h;
        if (ret.u >= 0 && ret.u <= 1 && ret.v >= 0 && ret.v <= 1) {
            ret.material = material;
            if (ret.type == OUTFROM) ret.normal = ret.normal * -1, ret.type = INTO;
            Vector3f normal_texture = material->get_normal(ret.u, ret.v);
            // std::cout << "normal_texture = " << normal_texture << std::endl;
            ret.normal = normal_texture.x * right - normal_texture.y * up + normal_texture.z * ret.normal;
            return ret;
        }
        return Intersection();
    }

    friend std::istream &operator >>(std::istream &fin, Rectangle &rectangle);
};

std::istream &operator >>(std::istream &fin, Rectangle &rectangle) {
    std::string s;
    while (fin >> s) {
        if (s.length() == 0 || s[0] == '#') {
            fin.ignore(256, '\n');
            continue;
        }
        if (s == "end") break;
        else if (s == "position") fin >> rectangle.pos;
        else if (s == "up") fin >> rectangle.up;
        else if (s == "right") fin >> rectangle.right;
        else if (s == "material") {
            rectangle.material.reset(new Material());
            fin >> *(rectangle.material);
        }
        else {
            std::cerr << std::string("Error: Unrecognized rectangle parameter : ") + s << std::endl;
            exit(-1);
        }
    }
    Vector3f normal = cross(rectangle.right, rectangle.up).normalized();
    rectangle.plane = Plane(normal, dot(rectangle.pos, normal));
    rectangle.w = rectangle.right.norm(), rectangle.right = rectangle.right * (1. / rectangle.w);
    rectangle.h = rectangle.up.norm(), rectangle.up = rectangle.up * (1. / rectangle.h);
    return fin;
}
