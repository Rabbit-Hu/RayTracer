#pragma once
#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include <memory>
#include "Vectors.h"
#include "config.h"
#include "Object.h"

class Triangle: public Object {
public:
    Vector3f v[3], vn[3];
    Vector2f vt[3];
    bool has_vn = false;
    
    Triangle(): Object(TRIANGLE) {}
    Triangle(Vector3f v0,  Vector3f v1,  Vector3f v2, 
             std::shared_ptr<Material> _material): Object(TRIANGLE, _material), has_vn(false) {
        v[0] = v0, v[1] = v1, v[2] = v2;
    }
    Triangle(Vector3f v0,  Vector3f v1,  Vector3f v2, 
             Vector2f vt0, Vector2f vt1, Vector2f vt2,
             Vector3f vn0, Vector3f vn1, Vector3f vn2, 
             std::shared_ptr<Material> _material): Object(TRIANGLE, _material), has_vn(true) {
        v[0]  = v0,  v[1]  = v1,  v[2]  = v2;
        vt[0] = vt0, vt[1] = vt1, vt[2] = vt2;
        vn[0] = vn0, vn[1] = vn1, vn[2] = vn2;
    }
    ~Triangle() {}

    double det3(const Vector3f& a, const Vector3f& b, const Vector3f& c) const {
		return dot(cross(a, b), c);
	}
    Intersection intersect(const Ray &ray, unsigned short *Xi) const override {
        Intersection ret;
		Vector3f E1 = v[0] - v[1], E2 = v[0] - v[2], S = v[0] - ray.o;
		double normalize_term = 1 / det3(ray.d, E1, E2);
		double t = normalize_term * det3(S, E1, E2);
		double beta = normalize_term * det3(ray.d, S, E2);
		double gamma = normalize_term * det3(ray.d, E1, S);
		if (t > 0 && beta >= 0 && beta <= 1 && gamma >= 0 && gamma <= 1 && beta + gamma <= 1){
            double alpha = 1 - beta - gamma;
			if (!has_vn) {
                ret.normal = cross(v[1] - v[0], v[2] - v[0]).normalized(); 
                // std::cout << "no vertex normal!" << std::endl;
            }
            else {
                ret.normal = (alpha * vn[0] + beta * vn[1] + gamma * vn[2]).normalized(); // smooth shading
                // ret.normal = Vector3f(0, 0, 1);
                // std::cout << "vetex normals = " << vn[0] << vn[1] << vn[2] << std::endl;
                // std::cout << "  ret.normal  = " << ret.normal << std::endl;
            }
            ret.type = dot(ray.d, ret.normal) < 0 ? INTO : OUTFROM;
            ret.t = t;
            ret.poc = ray.o + t * ray.d;
            ret.material = material;
            Vector2f uv = alpha * vt[0] + beta * vt[1] + gamma * vt[2];
            ret.u = uv.x, ret.v = uv.y;
        }
        // std::cout << "intersection (triangle) = " << ret << std::endl;
        // if (ret.type != MISS) num_triangle_intersects++;
        return ret;
    }
};
