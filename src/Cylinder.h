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

class Cylinder: public Object {
public:
    Vector3f pos, up, right = Vector3f(0, 0, 0); // right is currently not used
    double rad, y_min, y_max;
    
    Cylinder(): Object(CYLINDER) {}
    Cylinder(Vector3f _pos, Vector3f _up, Vector3f _right, double _rad, double _y_min, double _y_max): 
        Object(CYLINDER), pos(_pos), up(_up), right(_right), rad(_rad), y_min(_y_min), y_max(_y_max) {}
    ~Cylinder() {}

    Intersection intersect(const Ray &ray) const override {
        Intersection ret;
        Vector3f o_minus_pos = ray.o - pos;
        double d_dot_up = dot(ray.d, up), o_minus_pos_dot_up = dot(o_minus_pos, up);
        if (abs(d_dot_up - 1) > EPS) { // intersect with side
            double a = dot(ray.d, ray.d) - d_dot_up * d_dot_up;
            double b = 2 * (dot(o_minus_pos, ray.d) - o_minus_pos_dot_up * d_dot_up);
            double c = dot(o_minus_pos, o_minus_pos) - o_minus_pos_dot_up * o_minus_pos_dot_up - rad * rad;
            double delta = b * b - 4 * a * c;
            if (delta > EPS) {
                delta = sqrt(delta);
                if ((ret.t = -b - delta) > EPS || (ret.t = -b + delta) > EPS) {
                    ret.t /= 2 * a;
                    ret.poc = ray.o + ret.t * ray.d;
                    Vector3f poc_minus_pos = ret.poc - pos;
                    double poc_minus_pos_dot_up = dot(ret.poc - pos, up);
                    if (poc_minus_pos_dot_up < y_max && poc_minus_pos_dot_up > y_min) {
                        ret.normal = (poc_minus_pos - poc_minus_pos_dot_up * up).normalized();
                        ret.type = dot(ret.normal, ray.d) < 0 ? INTO: OUTFROM;
                    }
                }
            }
        }
        if (ret.type == MISS) ret.t = INF_D; 
        Plane max_plane(up, dot(pos, up) + y_max), min_plane(-1 * up, -(dot(pos, up) + y_min));
        Intersection max_its = max_plane.intersect(ray), min_its = min_plane.intersect(ray);
        if (max_its.t < ret.t && (max_its.poc - (pos + y_max * up)).norm() < rad) ret = max_its;
        if (min_its.t < ret.t && (min_its.poc - (pos + y_min * up)).norm() < rad) ret = min_its;
        ret.material = material;
        return ret;
    }

    friend std::istream &operator >>(std::istream &fin, Cylinder &cylinder);
};

std::istream &operator >>(std::istream &fin, Cylinder &cylinder) {
    std::string s;
    while (fin >> s) {
        if (s.length() == 0 || s[0] == '#') {
            fin.ignore(256, '\n');
            continue;
        }
        if (s == "end") break;
        else if (s == "position") fin >> cylinder.pos;
        else if (s == "up") fin >> cylinder.up;
        else if (s == "y_min") fin >> cylinder.y_min;
        else if (s == "y_max") fin >> cylinder.y_max;
        else if (s == "material") {
            cylinder.material.reset(new Material());
            fin >> *(cylinder.material);
        }
        else if (s == "radius") fin >> cylinder.rad;
        else {
            std::cerr << std::string("Error: Unrecognized cylinder parameter : ") + s << std::endl;
            exit(-1);
        }
    }
    return fin;
}
