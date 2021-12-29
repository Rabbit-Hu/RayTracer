#include <iostream>
#include <cmath>
#include "src/Vectors.h"
#include "src/Canvas.h"
#define MAX_HUE 0.8
#define MIN_WAVELENGTH 0.440
#define MAX_WAVELENGTH 0.630

Vector3f hsv2rgb(const Vector3f &hsv) {
    // h, s, v in [0, 1]
    double h = hsv.x, s = hsv.y, v = hsv.z;
    double c = v * s, x = c * (1 - fabs(fmod(h * 6, 2) - 1)), m = v - c;
    Vector3f rgb;
    if (6 * h < 1) rgb = Vector3f(c, x, 0);
    else if (6 * h < 2) rgb = Vector3f(x, c, 0);
    else if (6 * h < 3) rgb = Vector3f(0, c, x);
    else if (6 * h < 4) rgb = Vector3f(0, x, c);
    else if (6 * h < 5) rgb = Vector3f(x, 0, c);
    else rgb = Vector3f(c, 0, x);
    return rgb + Vector3f(m, m, m);
}
Vector3f rgb2hsv(const Vector3f &rgb) {
    double r = rgb.x, g = rgb.y, b = rgb.z;
    double c_max = fmax(fmax(r, g), b), c_min = fmin(fmin(r, g), b), delta = c_max - c_min;
    double h;
    if (delta < EPS) h = 0;
    else if (c_max == r) h = fmod((g - b) / delta + 6, 6);
    else if (c_max == g) h = (b - r) / delta + 2;
    else h = (r - g) / delta + 4;
    return Vector3f(h / 6, c_max < EPS ? 0: delta/c_max, c_max);
}
Vector2f rgb2wavelength_fake(const Vector3f rgb) {
    // return: (wavelength, value). Wave lengths are in micrometer (0.440 ~ 0.630)
    Vector3f hsv = rgb2hsv(rgb);
    double h = hsv.x, s = hsv.y, v = hsv.z;
    if (h > MAX_HUE || drand48() > s) h = drand48();
    else h /= MAX_HUE;
    return Vector2f(MAX_WAVELENGTH - h * (MAX_WAVELENGTH - MIN_WAVELENGTH), v);
}
Vector3f wavelength2rgb_fake(const Vector2f wave) {
    return hsv2rgb(Vector3f(MAX_HUE * (MAX_WAVELENGTH - wave.x) / (MAX_WAVELENGTH - MIN_WAVELENGTH), 1, wave.y));
}
double glass_refractive_index(double wavelength) {
    // reference: https://refractiveindex.info/?shelf=3d&book=glass&page=BK7
    wavelength = wavelength * wavelength;
    return sqrt(1 + 1.04*wavelength/(wavelength-0.006) + 0.23*wavelength/(wavelength-0.02) + 1.01*wavelength/(wavelength-103.56));
}