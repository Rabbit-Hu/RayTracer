#include <iostream>
#include <cmath>
#include <algorithm>
#include "src/Vectors.h"
#include "src/Canvas.h"
#include "src/Color.h"
using namespace std;

bool l1_cmp(Vector3f a, Vector3f b) {
    return a.x + a.y + a.z < b.x + b.y + b.z;
}

int main(){

    int filter_size = 3;

    Canvas input;
    input.read_png("/root/RayTracer/output/wallpaper.png");
    int w = input.width(), h = input.height();
    Canvas output(w, h);

    int filter_radius = filter_size / 2;
    Vector3f neighbors[filter_size * filter_size];
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int cnt = 0;
            for (int i = max(-filter_radius, -y); i <= filter_radius && y + i < h; i++)
                for (int j = max(-filter_radius, -x); j <= filter_radius && x + j < w; j++) 
                    if (i * i + j * j <= filter_radius * filter_radius) 
                        neighbors[cnt++] = input[y + i][x + j];
            nth_element(neighbors, neighbors + cnt / 2, neighbors + cnt, l1_cmp);
            output[y][x] = neighbors[cnt / 2];
        }
    }
    output.write_png("../output/filtered.png");

    return 0;
}