#pragma once
#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include <cassert>
#include "Vectors.h"
#include "Object.h"
#include "config.h"
#include "Cylinder.h"
#define NEWTON_ITER 8
#define NEWTON_ATTEMPT 30
#define NEWTON_DELTA 1e-6


class BezierCurve2D {
public:
    std::vector<Vector2f> control;
    double y_min = INF_D, y_max = -INF_D, x_max = 0; // 2D AABB
    BezierCurve2D(){}
    BezierCurve2D(int size): control(size){}
    BezierCurve2D(const std::vector<Vector2f> &_control): control(_control){}

    int size() const {
        return control.size();
    }

    BezierCurve2D operator*(double b) const {
        BezierCurve2D ret(control);
        for(int i = 0; i < ret.control.size(); i++) 
            ret.control[i] = ret.control[i] * b;
        ret.y_min = y_min * b, ret.y_max = y_max * b, ret.x_max = x_max * b;
        return ret;
    }

    BezierCurve2D operator+(const BezierCurve2D &b) const {
        BezierCurve2D ret(control);
        assert(control.size() == b.control.size());
        for(int i = 0; i < ret.control.size(); i++) 
            ret.control[i] = ret.control[i] + b.control[i];
        ret.y_min = y_min + b.y_min, ret.y_max = y_max + b.y_max, ret.x_max = x_max + b.x_max;
        return ret;
    }

    void getAabb() {
        int size = control.size();
        y_min = fmin(control[0].y, control[size-1].y);
        y_max = fmax(control[0].y, control[size-1].y);
        x_max = fmax(control[0].x, control[size-1].x);

        if (size <= 2) return;
        
        // find all the zero points of derivative curve
        BezierCurve2D reduced(size - 1);
        for (int i = 0; i < size - 1; i++)
            reduced.control[i] = (control[i + 1] - control[i]) * (size - 1);
        // x_max
        for (int q = 0; q < NEWTON_ATTEMPT; q++) {
            double t = drand48();
            for (int i = 0; i < NEWTON_ITER && t >= 0 && t <= 1; i++) {
                double f = reduced.eval(t).x;
                if (fabs(f) < NEWTON_DELTA) {
                    x_max = fmax(x_max, this->eval(t).x);
                    break;
                }
                double d = reduced.deri(t).x;
                t -= f / d;
            }
        }
        // y_min and y_max
        for (int q = 0; q < NEWTON_ATTEMPT; q++) {
            double t = drand48();
            for (int i = 0; i < NEWTON_ITER && t >= 0 && t <= 1; i++) {
                double f = reduced.eval(t).y;
                if (fabs(f) < NEWTON_DELTA) {
                    double y_new = this->eval(t).y;
                    y_max = fmax(y_max, y_new);
                    y_min = fmin(y_min, y_new);
                    break;
                }
                double d = reduced.deri(t).y;
                t -= f / d;
            }
        }
    }

    Vector2f eval(double t) const {
        if (control.size() == 1)
            return control[0];
        int size = control.size();
        BezierCurve2D reduced(size - 1);
        for (int i = 0; i < size - 1; i++)
            reduced.control[i] = control[i] * (1 - t) + control[i + 1] * t;
        return reduced.eval(t);
    }

    Vector2f deri(double t) const {
        int size = control.size();
        BezierCurve2D reduced(size - 1);
        for (int i = 0; i < size - 1; i++)
            reduced.control[i] = (control[i + 1] - control[i]) * (size - 1);
        return reduced.eval(t);
    }
};

class BezierRotational: public Object {
public:
    Vector3f pos = Vector3f(0, 0, 0), up = Vector3f(0, 0, 1), right = Vector3f(0, 1, 0); // rotate around "up"; use "right" for uv mapping
    BezierCurve2D end_curve, start_curve; // make sure that curve.control[0].y < curve.control[1].y
    double scale = 1, blur = 0; // real_size = scale * curve_size
    
    BezierRotational(): Object(BEZIER) {}
    ~BezierRotational() {}

    Vector3f eval(double u, double v, const BezierCurve2D &curve) const { // theta = 2*PI*u, t = v
        Vector2f point2D = curve.eval(v);
        Vector3f point3D = pos + scale * (point2D.x * right.rotate(up, 2*PI*u) + point2D.y * up);
        // std::cout << "point3D = " << point3D << std::endl;
        return point3D;
    }
    // Vector3f eval(double u, double v) const { // theta = 2*PI*u, t = v
    //     return eval(u, v, end_curve);
    // }

    Vector3f deri_u(double u, double v, const BezierCurve2D &curve) const {
        Vector3f d = right.rotate(up, 2*PI*u + PI/2);
        double r = curve.eval(v).x * scale;
        return 2 * PI * r * d;
    }
    // Vector3f deri_u(double u, double v) const {
    //     return deri_u(u, v, end_curve);
    // }

    Vector3f deri_v(double u, double v, const BezierCurve2D &curve) const {
        Vector2f dv2D = curve.deri(v);
        return scale * (dv2D.x * right.rotate(up, 2*PI*u) + dv2D.y * up);
    }
    // Vector3f deri_v(double u, double v) const {
    //     return deri_v(u, v, end_curve);
    // }

    static Vector3f gauss_solve(Vector3f a0, Vector3f a1, Vector3f a2, Vector3f b) {
        // (a0, a1, a2) * x = b, solve x
        double a[3][4] = {
            {a0.x, a1.x, a2.x, b.x},
            {a0.y, a1.y, a2.y, b.y},
            {a0.z, a1.z, a2.z, b.z},
        };
        for (int i = 0; i < 3; i++) {
            int best_row = i;
            for (int j = i + 1; j < 3; j++)
                if (fabs(a[j][i]) > fabs(a[best_row][i]))
                    best_row = j;
            if (i != best_row)
                for (int j = i; j < 4; j++)
                    std::swap(a[i][j], a[best_row][j]);
            for(int j = i + 1; j < 3; j++) {
                double ratio = a[j][i] / a[i][i];
                for (int k = i; k < 4; k++) {
                    a[j][k] -= ratio * a[i][k];
                }
            }
        }
        for (int i = 2; i >= 0; i--) {
            for (int j = i + 1; j < 3; j++)
                a[i][3] -= a[j][3] * a[i][j];
            // if(fabs(a[i][i]) > EPS) 
            a[i][3] /= a[i][i];
        }
        return Vector3f(a[0][3], a[1][3], a[2][3]);
    }

    Intersection intersect(const Ray &ray) const override {
        double t = exp(-1.0 * drand48() * 14 / blur); // log(1e-6) \approx 14
        return intersect_at_time(ray, t);
    }

    Intersection intersect_at_time(const Ray &ray, double t) const {
        BezierCurve2D curve = start_curve.size() ? end_curve * (1 - t) + start_curve * t : end_curve;
        // std::cout << "curve:     " << curve.x_max << " " << curve.y_min << " " << curve.y_max << " " << std::endl;
        // std::cout << "end_curve: " << end_curve.x_max << " " << end_curve.y_min << " " << end_curve.y_max << " " << std::endl;
        // std::cout << "start_curve: " << start_curve.x_max << " " << start_curve.y_min << " " << start_curve.y_max << " " << std::endl;

        // test intersection with the bounding cylinder, radius = curve.x_max;
        Cylinder cyl(pos, up, right, curve.x_max * scale, curve.y_min * scale, curve.y_max * scale);
        Intersection cyl_its = cyl.intersect(ray);
        if (cyl_its.type == MISS) return Intersection();
        Vector3f flat = cyl_its.poc - pos;
        flat = flat - dot(flat, up) * up;
        double cyl_its_angle = atan2(dot(cross(right, up), flat), dot(right, flat));

        Intersection ret;
        for (int q = 0; q < NEWTON_ATTEMPT; q++) {
            // double t = (pos - ray.o).norm() * (1 + (drand48() - 0.5) * 0.2), u = drand48(), v = drand48();
            // double t = cyl_its.t * (1 + (drand48() - 0.5) * 0.6), u = cyl_its_angle / (2*PI) + (drand48() - 0.5) * 0.6, v = drand48();
            double t = cyl_its.t * (1 + (drand48() - 0.5) * 0.2), u = cyl_its_angle / (2*PI) + (drand48() - 0.5) * 0.2, v = drand48();
            for (int i = 0; i < NEWTON_ITER; i++) {
                v = fmod(v, 2);
                if (v < 0) v += 2;
                if (v > 1) v = 2 - v, u += 0.5;
                u = fmod(u, 1); // 0 is equivalent to 2*PI
                if (u < 0) u += 1;
                Vector3f p = eval(u, v, curve), f = ray.o + ray.d * t - p, du = deri_u(u, v, curve), dv = deri_v(u, v, curve);
                // if (t < EPS || v < 0 || v > 1) break;
                if (t > EPS && f.norm() < NEWTON_DELTA) {
                    if (t < ret.t && curve.eval(v).x > 0) { // crossing the rotation axis is not allowed!
                        ret.t = t, ret.u = u, ret.v = v, ret.poc = p;
                        ret.normal = cross(du, dv).normalized();
                        ret.type = dot(ray.d, ret.normal) > 0 ? OUTFROM : INTO;
                    }
                    break;
                }
                Vector3f d_tuv = gauss_solve(-1 * ray.d, du, dv, f); // (dt, du, dv) * d_tuv  = f
                t += d_tuv.x, u += d_tuv.y, v += d_tuv.z;
            }
        }
        // std::cout << std::endl;
        ret.material = material;
        // if (ret.type != MISS) std::cout << "Not missed!" << std::endl;
        return ret;
    }

    friend std::istream &operator >>(std::istream &fin, BezierRotational &bezier);
};

std::istream &operator >>(std::istream &fin, BezierRotational &bezier) {
    std::string s;
    while (fin >> s) {
        if (s.length() == 0 || s[0] == '#') {
            fin.ignore(256, '\n');
            continue;
        }
        if (s == "end") break;
        else if (s == "c") {
            Vector2f c;
            fin >> c;
            bezier.end_curve.control.push_back(c);
        }
        else if (s == "from") {
            Vector2f c;
            fin >> c;
            bezier.start_curve.control.push_back(c);
        }
        else if (s == "position") {
            fin >> bezier.pos;
        }
        else if (s == "up") {
            fin >> bezier.up;
            bezier.up = bezier.up.normalized();
        }
        else if (s == "right") {
            fin >> bezier.right;
            bezier.right = bezier.right.normalized();
        }
        else if (s == "scale") {
            fin >> bezier.scale;
        }
        else if (s == "blur") {
            fin >> bezier.blur;
        }
        else if (s == "material") {
            bezier.material.reset(new Material());
            fin >> *(bezier.material);
        }
        else {
            std::cerr << std::string("Error: Unrecognized bezier parameter : ") + s << std::endl;
            exit(-1);
        }
    }
    int n = bezier.end_curve.control.size();
    if (n < 2) {
        std::cerr << "Error: Bezier curve requires at least 2 control points, but only read " << n << "point(s)" <<  std::endl;
        exit(-1);
    }
    if (bezier.end_curve.control.begin()->y > bezier.end_curve.control.rbegin()->y) {
        std::reverse(bezier.end_curve.control.begin(), bezier.end_curve.control.end());
        if (bezier.start_curve.size()) std::reverse(bezier.start_curve.control.begin(), bezier.start_curve.control.end());
    }
    bezier.end_curve.getAabb();
    if (bezier.start_curve.size()) bezier.start_curve.getAabb();
    return fin;
}