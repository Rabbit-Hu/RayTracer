#include <iostream>
#include <cmath>
#include <ctime>
#include "src/Vectors.h"
#include "src/Canvas.h"
#include "src/Color.h"


int main(){

    int w = 360, h = 30;
    Canvas canvas(w, h);
    double sat = 1, val = 1;
    for (int x = 0; x < w; x++) {
        double hue = (double) x / w;
        for (int y = 0; y < h; y++) 
            canvas[y][x] = wavelength2rgb_fake(rgb2wavelength_fake(hsv2rgb(Vector3f(hue, sat, val))));
    }
    canvas.write_png("../output/spectrum.png");

    // srand48(time(0));
    // Vector3f hsv(drand48(), drand48(), drand48());
    // std::cout << hsv << ", " << rgb2hsv(hsv2rgb(hsv)) << std::endl;

    // Vector3f rgb(0.8, 0.2, 0.5), hsv = rgb2hsv(rgb);
    // Vector2f wave = rgb2wavelength_fake(rgb);
    // std::cout << hsv << ", " << wave << std::endl;

    return 0;
}