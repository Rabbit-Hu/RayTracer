#pragma once
#include "Camera.h"
#include "Canvas.h"
#include "Object.h"
#include "Vectors.h"
#include "Sphere.h"
#include "Cylinder.h"
#include "Plane.h"
#include "Triangle.h"
#include "Mesh.h"
#include "Bezier.h"
#include "Prism.h"
#include "Color.h"
#include <cmath>
#include <iostream>
#include <vector>
#include <chrono>

class Scene {
public:
    Camera camera;
    std::vector<Object *> objects;
    int samp = 1000;

    Scene() {}
    Scene(std::string fname) {
        std::ifstream fin(fname.c_str());
        std::string s;
        bool _camera = false;
        while (fin >> s) {
            if (s.length() == 0 || s[0] == '#') {
                fin.ignore(256, '\n');
                continue;
            }
            else if (s == "camera") {
                fin >> camera;
                _camera = true;
            }
            else if (s == "samp") {
                fin >> samp;
            }
            else if (s == "sphere") {
                auto *object = new Sphere();
                fin >> *object;
                objects.push_back(object);
            }
            else if (s == "cylinder") {
                auto *object = new Cylinder();
                fin >> *object;
                objects.push_back(object);
            }
            else if (s == "plane") {
                auto *object = new Plane();
                fin >> *object;
                objects.push_back(object);
            }
            else if (s == "mesh") {
                fin >> s;
                auto *object = new Mesh(s);
                objects.push_back(object);
            }
            else if (s == "bezier") {
                auto *object = new BezierRotational();
                fin >> *object;
                objects.push_back(object);
            }
            else if (s == "prism") {
                auto *object = new Prism();
                fin >> *object;
                objects.push_back(object);
            }
            else {
                std::cerr << std::string("Error: Unrecognized object : ") + s << std::endl;
                exit(-1);
            }
        }
        if (!_camera) {
            std::cerr << std::string("Error: Camera not found") << std::endl;
            exit(-1);
        }
    }
    ~Scene() {
        for (auto object : objects)
            delete object;
    }

    Intersection intersect(const Ray &ray) {
        Intersection ret;
        for (auto &object : objects) {
            Intersection tmp = object->intersect(ray);
            if (tmp.type != MISS && tmp.t < ret.t){
                ret = tmp;
                // std::cout << "ret.t = " << ret.t << ", Kd = " << ret.material->Kd << std::endl;
            }
        }
        // std::cout << "intersection = " << ret << std::endl;
        return ret;
    }

    Vector3f random_hemi_ray_cos(const Vector3f &normal, unsigned short *Xi) {
        // generate random vector obeying cosine distribution
        Vector3f xx;
        if (fabs(normal.x) > EPS) xx = Vector3f(normal.y, -normal.x, 0).normalized();
        else xx = Vector3f(0, -normal.z, normal.y).normalized();
        Vector3f yy = cross(normal, xx);
        // a formula for random direction
        // https://cg.informatik.uni-freiburg.de/course_notes/graphics2_08_renderingEquation.pdf
        double theta = 2 * PI * erand48(Xi);
        double r = erand48(Xi);
        double sr = sqrt(r);
        return (xx * sin(theta) * sr + yy * cos(theta) * sr + normal * sqrt(1 - r)).normalized();
    }

    Vector3f radiance(const Ray &ray, int depth, unsigned short *Xi) {
        // std::cout << "ray = " << ray << std::endl;
        Intersection its = intersect(ray);
        if (its.type == MISS) return Vector3f();
        if (depth >= REFLECT_LIMIT) return its.material->Ke;
        Vector3f normal = its.type == INTO ? its.normal : -1 * its.normal;
        // std::cout << "intersection = " << its << std::endl;
        Vector3f emit_color = its.material->get_emit(its.u, its.v), 
                 diffuse_color = its.material->get_diffuse(its.u, its.v),
                 specular_color = its.material->get_specular(),
                 absorb_color = its.material->get_absorb();
        Vector3f color = emit_color;
        double Ni = its.material->Ni;

        // std::cout << "diffuse_color = " << diffuse_color << std::endl;
        // return diffuse_color; // uncomment this for testing...

        // Refraction and Reflection
        if (Ni > EPS) {
            Ray reflect = Ray(its.poc, (ray.d - normal * 2 * dot(normal, ray.d)).normalized());
            double eta = its.type == INTO ? 1 / Ni : Ni; // sin(r)/sin(i)
            double cos_i = dot(normal, ray.d); // Actually it's -cos(i)
            double cos_r_square = 1 - eta * eta * (1 - cos_i * cos_i);
            if (cos_r_square < EPS) {
                return emit_color + get_radiance_with_coeff(reflect, depth + 1, Xi, diffuse_color);
            }
            Ray refract = Ray(its.poc, (ray.d * eta - normal * (cos_i * eta + sqrt(cos_r_square))).normalized());
            double a = Ni - 1, b = Ni + 1;
            double R0 = (a * a) / (b * b), c = 1 - (its.type == INTO ? -cos_i : dot(refract.d, its.normal));
            double Re = R0 + (1 - R0) * c * c * c * c * c, Tr = 1 - Re, P = 0.25 + 0.5 * Re, RP = Re / P, TP = Tr / (1 - P);
            // TODO: explain this with physics
            // Vector3f to_add = specular_color * (Vector3f(1,1,1) - diffuse_color), alpha = specular_color * (Vector3f(1,1,1) - diffuse_color); 
            Vector3f to_add = specular_color, alpha = specular_color; 
            if (its.type != INTO) alpha = pow(absorb_color, its.t);
            // if (depth < 1) to_add = radiance(reflect, depth + 1, Xi) * Re * to_add + radiance(refract, depth + 1, Xi) * Tr * alpha;
            if (depth < 1) {
                to_add = get_radiance_with_coeff(reflect, depth + 1, Xi, Re * to_add) 
                    + get_radiance_with_coeff(refract, depth + 1, Xi, Tr * alpha);
            }
            else if (erand48(Xi) < P) to_add = get_radiance_with_coeff(reflect, depth + 1, Xi, to_add * RP);
            else to_add = get_radiance_with_coeff(refract, depth + 1, Xi, TP * alpha);
            color = color + to_add;
            // std::cout << "to_add = " << to_add << std::endl;
        }

        // specular (My brain must have been short-circuited)
        // Ray reflect = Ray(its.poc, (ray.d - normal * 2 * dot(normal, ray.d)).normalized());
        // double specular_coeff = pow(fmax(0., -dot(ray.d, reflect.d)), shininess);
        // if (specular_coeff > EPS)
        //     color = color + specular_color * radiance(reflect, depth + 1, Xi) * diffuse_color;

        // diffuse
        // std::cout << "normal = " << normal << std::endl;
        Ray shadow = Ray(its.poc, random_hemi_ray_cos(normal, Xi));
        double SHADOW_EPS = 0.001;
        shadow.o = shadow.o + SHADOW_EPS * shadow.d;
        // shadow. d = Vector3f(0, 0, 1);// debug
        // std::cout << "normal = " << normal << ", shadow_ray = " << shadow << std::endl;
        // Vector3f shadow_color = radiance(shadow, depth + 1, Xi);
        // color = color + diffuse_color * shadow_color;
        Vector3f shadow_color = get_radiance_with_coeff(shadow, depth + 1, Xi, diffuse_color);
        color = color + shadow_color;
        // std::cout << "shadow_color = " << shadow_color << ", diffuse_color = " << diffuse_color << std::endl;

        return color;
    }
    Vector3f get_radiance_with_coeff(const Ray &ray, int depth, unsigned short *Xi, double coeff) {
            if (fabs(coeff) < EPS) return Vector3f();
            return radiance(ray, depth, Xi) * coeff;
    }
    Vector3f get_radiance_with_coeff(const Ray &ray, int depth, unsigned short *Xi, Vector3f coeff) {
        if (coeff.inf_norm() < EPS) return Vector3f();
        return radiance(ray, depth, Xi) * coeff;
    }
    Canvas *ray_trace(int h1, int h2, int w1, int w2) {
        Canvas *canvas = new Canvas(w2 - w1, h2 - h1);
        for (int y = h1; y < h2; y++){
            auto start_time = std::chrono::high_resolution_clock::now();
            // std::cout << "begin y = " << y << std::endl;
            unsigned short Xi[3]={0,0,0};
            Xi[2] = y*y*y;
            for (int x = w1; x < w2; x++){
                // std::cout << "x = " << x << std::endl;
                Vector3f color;
                for (int s = 0; s < samp; s++) {
                    for (int sx = 0; sx < 2; sx++)
                        for (int sy = 0; sy < 2; sy++) {
                            double r1 = 2 * erand48(Xi), dx = r1 < 1 ? sqrt(r1) - 1: 1 - sqrt(2 - r1);
                            double r2 = 2 * erand48(Xi), dy = r2 < 1 ? sqrt(r2) - 1: 1 - sqrt(2 - r2);
                            Ray ray = camera.get_ray((sx + 0.5 + dx)/2 + x, (sy + 0.5 + dy)/2 + y);
                            color = color + radiance(ray, 0, Xi) * (1. / samp) * .25;
                        }
                }
                (*canvas)[y - h1][x - w1] = color;
                // std::cout << "canvas[" << y << "][" << x << "] = " << color << std::endl;
            }

            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::ratio<1, 1> > time_duration = end_time - start_time;
            std::cout << "y = " << y << ", duration = " << time_duration.count() << "s" << std::endl;
        }
        return canvas;
    }
    Canvas *ray_trace() {
        return ray_trace(0, camera.h, 0, camera.w);
    }
};