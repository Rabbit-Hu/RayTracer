#pragma once
#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include <memory>
#include "Vectors.h"
#include "config.h"
#include "Object.h"
#include "Triangle.h"

class Prism: public Object {
public:
    Vector3f v[3];
    double height;

    Prism(): Object(PRISM) {}
    ~Prism() {}

    Intersection intersect(const Ray &ray) const override {
        Intersection ret, its;
        Vector3f up = cross(v[1] - v[0], v[2] - v[0]).normalized();
        Vector3f v_[3] = {v[0] + up * height, v[1] + up * height, v[2] + up * height};
        if ((its = Triangle( v[2],  v[1],  v[0], material).intersect(ray)).t < ret.t) ret = its;
        if ((its = Triangle(v_[0], v_[1], v_[2], material).intersect(ray)).t < ret.t) ret = its;
        if ((its = Triangle( v[0],  v[1], v_[1], material).intersect(ray)).t < ret.t) ret = its;
        if ((its = Triangle( v[1],  v[2], v_[2], material).intersect(ray)).t < ret.t) ret = its;
        if ((its = Triangle( v[2],  v[0], v_[0], material).intersect(ray)).t < ret.t) ret = its;
        if ((its = Triangle( v[1], v_[2], v_[1], material).intersect(ray)).t < ret.t) ret = its;
        if ((its = Triangle( v[2], v_[0], v_[2], material).intersect(ray)).t < ret.t) ret = its;
        if ((its = Triangle( v[0], v_[1], v_[0], material).intersect(ray)).t < ret.t) ret = its;
        return ret;
    }
    friend std::istream &operator >>(std::istream &fin, Prism &prism);
};

std::istream &operator >>(std::istream &fin, Prism &prism) {
    std::string s;
    int cnt = 0;
    while (fin >> s) {
        if (s.length() == 0 || s[0] == '#') {
            fin.ignore(256, '\n');
            continue;
        }
        else if (s == "end") break;
        else if (s == "height") fin >> prism.height;
        else if (s == "v") {
            fin >> prism.v[cnt++];
        }
        else if (s == "material") {
            prism.material.reset(new Material());
            fin >> *(prism.material);
        }
        else {
            std::cerr << std::string("Error: Unrecognized prism parameter : ") + s << std::endl;
            exit(-1);
        }
    }
    return fin;
}

