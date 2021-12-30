#pragma once
#include <cmath>
#include <string>
#include "config.h"
#include "png++/image.hpp"
#include "png++/require_color_space.hpp"

class Canvas {
private:
    int w, h;
    Vector3f *colors;
    double clamp(double x, double low=0., double high=1.) {
        return x > high ? high : (x < low ? low : x);
    }
    int d_to_byte(double d, double gamma=1) { // gamma correction
        return int(pow(clamp(d), 1/gamma) * 255 + 0.5);
    } // 2.2 is gamma correction, see https://en.wikipedia.org/wiki/Gamma_correction
    double byte_to_d(int b, double gamma=1) {
        return pow(clamp(b, 0, 255) / 255, gamma);
    }
public:
    Canvas(): colors(nullptr) {}
    Canvas(int _w, int _h): w(_w), h(_h) {
        colors = new Vector3f[w * h];
    }
    ~Canvas(){
        delete[] colors;
    }
    int width(){ return w; }
    int height(){ return h; }
    bool in_range(int y, int x) {
        return y >= 0 && y < h && x >= 0 && x < w;
    }
    Vector3f *operator[](int y) { // needed for writing pixels
        return colors + y * w;
    } // can use "canvas[y][x] = vec", may be convenient?
    Vector3f get_color(int y, int x) const { // if (x, y) out of range, jump to the opposite side
        y %= h;
        if (y < 0) y += h;
        x %= w;
        if (x < 0) x += w;
        return colors[y * w + x];
    }
    Vector3f get_color(double y, double x) const { // get color with interpolation
        // std::cout << " get_color(" << y << ", " << x << "), w = " << w << ", h = " << h << std::endl;
        int y_floor = floor(y), x_floor = floor(x);
        double y_res = y - y_floor, x_res = x - x_floor;
        // use interpolation for simplicity and efficiency
        return (1 - y_res) * (1 - x_res) * get_color(y_floor,     x_floor)     + 
               (1 - y_res) * x_res       * get_color(y_floor,     x_floor + 1) + 
               y_res *       (1 - x_res) * get_color(y_floor + 1, x_floor)     + 
               y_res *       x_res       * get_color(y_floor + 1, x_floor + 1);
    }
    Vector2f get_dxdy(double y, double x) const {
        return 0.5 * Vector2f(get_color(y, x + 1).x - get_color(y, x - 1).x, get_color(y + 1, x).x - get_color(y - 1, x).x);
    }
    Vector3f get_color_uv(double u, double v) const { // u, v in [0, 1], u horizontal, v vertical
        return get_color((1 - v) * h, u * w);
    }
    void set_submatrix(const Canvas &b, int y1, int x1) {
        for (int y = y1; y < y1 + b.h; y++)
            for (int x = x1; x < x1 + b.w; x++)
                colors[y * w + x] = b.colors[(y - y1) * b.w + (x - x1)];
    }
    void read_png(const std::string &fname) {
        // std::cout << "Loading png file " << fname << " into Canvas ..." << std::endl;
        if (colors != nullptr) delete[] colors;
        png::image<png::rgb_pixel> img(fname.c_str(), png::require_color_space<png::rgb_pixel>());
        w = img.get_width();
        h = img.get_height();
        // std::cout << "w = " << w << ", h = " << h << std::endl;
        colors = new Vector3f[w * h];
        for (size_t y = 0; y < h; y++)
            for (size_t x = 0; x < w; x++){
                // std::cout << "x = " << x << ", y = " << y << std::endl;
                auto rgb = img.get_pixel(x, y);
                // std::cout << "rgb = (" << int(rgb.red) << ", " << int(rgb.green) << ", " << int(rgb.blue) << ")" << std::endl;
                (*this)[y][x] = Vector3f(byte_to_d(rgb.red), byte_to_d(rgb.green), byte_to_d(rgb.blue));
            }
        // std::cout << "Successfully loaded png file " << fname << "." << std::endl;
    }
    void write_png(const std::string &fname) {
        // std::cout << "Writing png file " << fname << " ..." << std::endl;
        png::image<png::rgb_pixel> img(w, h);
        for (size_t y = 0; y < h; y++)
            for (size_t x = 0; x < w; x++){
                auto color = (*this)[y][x];
                img[y][x] = png::rgb_pixel(d_to_byte(color.x), d_to_byte(color.y), d_to_byte(color.z));
            }
        img.write(fname);
    }
};