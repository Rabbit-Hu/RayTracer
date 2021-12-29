#pragma once
#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include <memory>
#include <vector>
#include <map>
#include <algorithm>
#include "Vectors.h"
#include "config.h"
#include "Canvas.h"

enum HitType { MISS, INTO, OUTFROM };
enum ObjectType { AABB, SPHERE, PLANE, CYLINDER, BEZIER, TRIANGLE, MESH, PRISM };

class Ray {
public:
    Vector3f o, d;
    Ray(Vector3f _o, Vector3f _d): o(_o), d(_d) {}

    // friend std::istream &operator >>(std::istream &fin, Ray &r);
    friend std::ostream &operator <<(std::ostream &fout, const Ray &r);
};

std::ostream &operator <<(std::ostream &fout, const Ray &r) {
    fout << "Ray(" << r.o << ", " << r.d << ")";
    return fout;
}

class Material {
public:
    /*
    Types of colors and their corresponding parameters:
        Diffuse:    color Kd; texture map map_Kd (optional).
        Emit:       color Ke; texture map map_Ke (optional).
        Specular:   color Ks;
        Absorb:     color Ka. Absorption when photon travels inside a transparent body. No map.
        Dispersion: dispersion like glass. 
    Others (not used): 
        Alpha value: d.
        Specular: color Ks (default white); shininess Ns. No map.
        illum (not used)
    */
    std::string file_dir = "";
    Vector3f Kd = Vector3f(1, 1, 1), Ke = Vector3f(0, 0, 0), Ks = Vector3f(1, 1, 1), Ka = Vector3f(0, 0, 0); // ambient, diffuse, specular, emit
    double Ni = 1.45; // density
    bool dispersion = false;
    Canvas *map_Kd = nullptr, *map_Ke = nullptr; // textures

    Material() {}
    ~Material() {
        if (map_Kd != nullptr) delete map_Kd; 
        if (map_Ke != nullptr) delete map_Ke;
    }

    Vector3f get_absorb() const {
        return Ka;
    }
    Vector3f get_diffuse(double u = 0, double v = 0) const {
        // if have texture map, Kd.x serves as coefficient
        if (map_Kd != nullptr) return Kd.x * map_Kd->get_color_uv(u, v);
        return Kd;
    }
    Vector3f get_emit(double u = 0, double v = 0) const {
        if (map_Ke != nullptr) return Ke.x * map_Ke->get_color_uv(u, v);
        return Ke;
    }   
    Vector3f get_specular() const {
        return Ks;
    }       

    friend std::istream &operator >>(std::istream &fin, Material &material);
};

std::istream &operator >>(std::istream &fin, Material &material) {
    if (material.map_Kd != nullptr) delete material.map_Kd; 
    if (material.map_Ke != nullptr) delete material.map_Ke;
    std::string s;
    while (fin >> s) {
        if (s.length() == 0 || s[0] == '#') {
            fin.ignore(256, '\n');
            continue;
        }
        else if (s == "illum" || s == "Ns" || s == "d") {// TODO
            fin.ignore(256, '\n');
            continue;
        }
        if (s == "end") break;
        else if (s == "Ka") fin >> material.Ka;
        else if (s == "Kd") fin >> material.Kd;
        else if (s == "Ks") fin >> material.Ks;
        else if (s == "Ke") fin >> material.Ke;
        else if (s == "Ni") fin >> material.Ni;
        else if (s == "map_Kd") {
            fin >> s;
            material.map_Kd = new Canvas();
            material.map_Kd->read_png(material.file_dir + s);
        }
        else if (s == "map_Ke") {
            fin >> s;
            material.map_Ke = new Canvas();
            material.map_Ke->read_png(material.file_dir + s);
        }
        else if (s == "newmtl") {
            for (int i = s.length() - 1; i >= 0; i--)
                fin.putback(s[i]);
            break;
        }
        else if (s == "dispersion") {
            int tmp;
            fin >> tmp;
            if (tmp) material.dispersion = true;
        }
        else {
            std::cerr << std::string("Error: Unrecognized material parameter : ") + s << std::endl;
            exit(-1);
        }
    }
    return fin;
}

class Intersection;
class Aabb;

class Object {
public:
    ObjectType type;
    std::shared_ptr<Material> material;
    // Aabb *aabb = nullptr;

    Object(ObjectType _type): type(_type) {}
    Object(ObjectType _type, std::shared_ptr<Material> _material): type(_type), material(_material) {}
    virtual ~Object() {};

    virtual Intersection intersect(const Ray &ray) const = 0;
};

class Intersection {
public:
    HitType type = MISS;
    double t = INF_D;
    Vector3f poc, normal; // poc: point of contact
    std::shared_ptr<Material> material;
    double u = 0, v = 0; // uv coordinates for texture (in Material)

    friend std::ostream &operator <<(std::ostream &fout, const Intersection &its);
};

std::ostream &operator <<(std::ostream &fout, const Intersection &its) {
    fout << "Intersection(t = " << its.t << ", normal = " << its.normal << ", u = " << its.u << ", v = " << its.v << ")";
    return fout;
}

class Aabb: public Object {
public:
    Vector3f p0, p1; // min point, max point
    
    Aabb(): Object(AABB), p0(INF_D, INF_D, INF_D), p1(-INF_D, -INF_D, -INF_D) {}
    Aabb(Vector3f _p0, Vector3f _p1): Object(AABB), p0(_p0), p1(_p1) {}
    ~Aabb() {}

    Intersection intersect(const Ray &ray) const override {
        double t_enter = -INF_D, t_exit = INF_D, t0, t1;
        // std::cout << "ray = " << ray;
        // if (std::abs(ray.d.x) > EPS) {
            t0 = (p0.x - ray.o.x) / ray.d.x, t1 = (p1.x - ray.o.x) / ray.d.x;
            t_enter = std::max(t_enter, std::min(t0, t1)), t_exit = std::min(t_exit, std::max(t0, t1));
        // }
        // if (std::abs(ray.d.y) > EPS) {
            t0 = (p0.y - ray.o.y) / ray.d.y, t1 = (p1.y - ray.o.y) / ray.d.y;
            t_enter = std::max(t_enter, std::min(t0, t1)), t_exit = std::min(t_exit, std::max(t0, t1));
        // }
        // if (std::abs(ray.d.z) > EPS) {
            t0 = (p0.z - ray.o.z) / ray.d.z, t1 = (p1.z - ray.o.z) / ray.d.z;
            t_enter = std::max(t_enter, std::min(t0, t1)), t_exit = std::min(t_exit, std::max(t0, t1));
        // }
        // std::cout << " t0 = " << t0 << " t1 = " << t1 << std::endl;
        if (t_enter > t_exit || t_exit < 0) return Intersection();
        Intersection ret;
        ret.type = INTO, ret.t = t_enter; 
        
        // ret.poc = ray.o + t_enter * ray.d;
        return ret;
    }
};
