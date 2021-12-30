#pragma once
#include <cmath>
#include "Vectors.h"
#include "Canvas.h"
#include "config.h"

class Camera {
public:
    Vector3f o, d, right, up;
    int w, h;
    double h_scale, w_scale, aperture, focus_distance;
    Camera() {}
    Camera(Vector3f _o, Vector3f _d, Vector3f _up, int _w, int _h, double _angle, Vector3f focus_pos, double _aperture): o(_o), d(_d), w(_w), h(_h), aperture(_aperture) {
        up = _up;
        up = (up - dot(up, d) * d).normalized();
        right = cross(d, up);
        double diagonal_scale = tanf(_angle / 2); // rad
        h_scale = diagonal_scale * h / sqrt(w*w+h*h);
        w_scale = diagonal_scale * w / sqrt(w*w+h*h);
        focus_distance = dot(d, focus_pos - o);
        // std::cout << "focus_distance = " << focus_distance << std::endl;
        aperture = aperture / 2;
    }
    Vector2f random_in_unit_disk(unsigned short *Xi) {
        Vector2f p;
        do {
            p = 2 * Vector2f(erand48(Xi), erand48(Xi)) - Vector2f(1, 1);
        } while (dot(p, p) > 1);
        return p;
    }
    Ray get_ray(double x, double y, unsigned short *Xi) { // x, y are pixel indices; dx
        double dx = (2.0 * (x + 0.5) / w - 1), dy = -(2.0 * (y + 0.5) / h - 1); // +0.5: center of pixel
        Vector2f offset = random_in_unit_disk(Xi) * aperture;
        Vector3f ray_o = o + right * offset.x + up * offset.y;
        // std::cout << "ray_o = " << ray_o << std::endl;
        Vector3f ray_d = (d + right * (dx * w_scale - offset.x / focus_distance) + up * (dy * h_scale - offset.y / focus_distance)).normalized();
        return Ray(ray_o, ray_d);
    }

    friend std::istream &operator >>(std::istream &fin, Camera &camera);
};

std::istream &operator >>(std::istream &fin, Camera &camera) {
    std::string s;
    Vector3f o, d, up, focus_pos;
    int w = 200, h = 200;
    double angle = 60, aperture = 0; // deg
    bool _o = false, _d = false, _up = false; // the input must contain these parameters
    while (fin >> s) {
        if (s.length() == 0 || s[0] == '#') {
            fin.ignore(256, '\n');
            continue;
        }
        if (s == "end") break;
        else if (s == "origin") {
            fin >> o;
            _o = true;
        }
        else if (s == "direction") {
            fin >> d;
            d = d.normalized();
            _d = true;
        }
        else if (s == "up") {
            fin >> up;
            up = up.normalized();
            _up = true;
        }
        else if (s == "width") {
            fin >> w;
        }
        else if (s == "height") {
            fin >> h;
        }
        else if (s == "angle") {
            fin >> angle;
        }
        else if (s == "aperture") {
            fin >> aperture;
        }
        else if (s == "focus_pos") {
            fin >> focus_pos;
        }
        else {
            std::cerr << std::string("Error: Unrecognized camera parameter : ") + s << std::endl;
            exit(-1);
        }
    }
    if (!_o || !_d || !_up) {
        std::string missing_str = !_o ? "origin" : (!_d ? "direction": "up");
        std::cerr << "Error: Camera parameter missing: " + missing_str << std::endl;
        exit(-1);
    }
    camera = Camera(o, d, up, w, h, angle / 180 * PI, focus_pos, aperture);
    return fin;
}
    