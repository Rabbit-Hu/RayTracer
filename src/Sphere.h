#pragma once
#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include <memory>
#include "Vectors.h"
#include "config.h"
#include "Canvas.h"


class Sphere: public Object {
public:
    Vector3f pos; // center
    double rad; // radius
    Vector3f up = Vector3f(0, 0, 1), right = Vector3f(0, 1, 0);
    
    Sphere(): Object(SPHERE) {}
    ~Sphere() {}

    Intersection intersect(const Ray &ray) const override {
        Intersection ret;
        Vector3f op = pos - ray.o;
        double b = dot(op, ray.d), t;
        double det = b*b + rad*rad - dot(op, op);
        if (det < 0) return Intersection();
        det = sqrt(det);
        if ((t = b - det) > EPS) ret.type = INTO;
        else if((t = b + det) > EPS) ret.type = OUTFROM;
        else return Intersection();
        ret.t = t;
        ret.poc = ray.o + t * ray.d;
        ret.normal = (ret.poc - pos).normalized();
        ret.material = material;
        ret.v = 1 - acos(dot(ret.normal, up)) / PI;
        Vector3f flat = ret.normal - up * dot(ret.normal, up);
        ret.u = atan2(dot(cross(right, flat), up), dot(right, flat)) / (2*PI);
        // std::cout << "up = " << up << ", right = " << right << ", sin = " << cross(right, flat).norm() << ", cos = " << dot(right, flat) << std::endl;
        return ret;
    }

    friend std::istream &operator >>(std::istream &fin, Sphere &sphere);
};

std::istream &operator >>(std::istream &fin, Sphere &sphere) {
    std::string s;
    bool _pos = false, _rad = false; // the input must contain these parameters
    while (fin >> s) {
        if (s.length() == 0 || s[0] == '#') {
            fin.ignore(256, '\n');
            continue;
        }
        if (s == "end") break;
        else if (s == "position") fin >> sphere.pos, _pos = true;
        else if (s == "up") fin >> sphere.up;
        else if (s == "right") fin >> sphere.right;
        else if (s == "material") {
            sphere.material.reset(new Material());
            fin >> *(sphere.material);
        }
        else if (s == "radius") fin >> sphere.rad, _rad = true;
        else {
            std::cerr << std::string("Error: Unrecognized sphere parameter : ") + s << std::endl;
            exit(-1);
        }
    }
    if (!(_pos && _rad)){
        std::string missing_str = !_pos ? "position" : "radius";
        std::cerr << "Error: Sphere parameter missing: " + missing_str << std::endl;
        exit(-1);
    }
    sphere.up = sphere.up.normalized();
    sphere.right = (sphere.right - sphere.up * dot(sphere.up, sphere.right)).normalized();
    return fin;
}
