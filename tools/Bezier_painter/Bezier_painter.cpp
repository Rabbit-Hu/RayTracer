#include <iostream>
#include <vector>
#include "../src/Vectors.h"
#include "../src/Canvas.h"
#include "../src/Bezier.h"


int main(){

    int w = 400, h = 400;
    Canvas canvas(w, h);
    Canvas mirrored(2 * w, h);
    
    // rotate around Y axis
    std::vector<Vector2f> control = {
        Vector2f(0, 0),
        Vector2f(150, 0),
        Vector2f(-400, 150),
        Vector2f(450, 170),
        Vector2f(180, 400),
        Vector2f(0, 400),
    };
    std::vector<Vector2f> from_control = {
        Vector2f(0, 0),
        Vector2f(100, 0),
        Vector2f(225, 0),
        Vector2f(115, 200),
        Vector2f(50, 200),
        Vector2f(0, 200),
    };
    // # c 0 2
    // # c 0.5 2
    // # c 1.15 2
    // # c 2.25 0
    // # c 1 0
    // # c 0 0
    
    BezierCurve2D curve(from_control);
    curve.getAabb();
    std::cout << "y_min = " << curve.y_min << ", y_max = " << curve.y_max << ", x_max = " << curve.x_max << std::endl;
    int t_samples = 1000;

    for (int t_idx = 0; t_idx < t_samples; t_idx++) {
        double t = (double)t_idx / (t_samples - 1);
        Vector2f p = curve.eval(t);
        int x = p.x + 0.5, y = h - p.y + 0.5;
        if (canvas.in_range(y, x)) {
            canvas[y][x] = Vector3f(1, 0, 0);
        }
    }
    if (int(curve.x_max + 0.5) >= 0 && int(curve.x_max + 0.5) < w)
        for (int y = fmax(h - curve.y_max + 0.5, 0); y < fmin(h - curve.y_min + 0.5, h); y++)
            canvas[y][int(curve.x_max + 0.5)] = Vector3f(0, 1, 0);
    canvas.write_png("../output/canvas.png");

    for (int x = 0; x < w; x++)
        for (int y = 0; y < h; y++)
            mirrored[y][w - x] = mirrored[y][w + x] = canvas[y][x];
    mirrored.write_png("../output/mirrored.png");

    return 0;
}