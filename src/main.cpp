#include <iostream>
#include <string>
#include <thread>
#include <algorithm>
#include "Vectors.h"
#include "Object.h"
#include "Scene.h"
#include "Bezier.h" // TODO: delete this


void draw_subcanvas(int num_workers, int worker, Scene *scene, Canvas *canvas) {
    int delta_h = (scene->camera.h + num_workers - 1) / num_workers;
    int h0 = worker * delta_h, h1 = std::min(scene->camera.h, (worker + 1) * delta_h);
    if (h0 >= h1) return;
    Canvas *subcanvas = scene->ray_trace(h0, h1, 0, scene->camera.w);
    canvas->set_submatrix(*subcanvas, h0, 0);
    delete subcanvas;
}

int main(int argc, char* argv[]) {
    srand48(20010313);

    std::string inputFile, outputFile;
    if (argc != 3) {
        std::cout << "Usage: ./RayTracer <input scene file> <output png file>" << std::endl;
        return 1;
        // inputFile = "../input/paimon.txt";
        // outputFile = "../output/paimon.png";
    }
    else {
        inputFile = argv[1];
        outputFile = argv[2];  // only png is allowed.
    }
    std::cout << "Input file: " << inputFile << std::endl;
    std::cout << "Output file: " << outputFile << std::endl << std::endl;

    Scene scene(inputFile);
    Canvas canvas(scene.camera.w, scene.camera.h);
    int num_workers = 8;
    std::thread *threads[num_workers];
    for(int i = 0; i < num_workers; i++) {
        threads[i] = new std::thread(draw_subcanvas, num_workers, i, &scene, &canvas);
    }
    for(int i = 0; i < num_workers; i++) {
        threads[i]->join();
        delete threads[i];
    }
    canvas.write_png(outputFile);
    
    // int h0 = 95./300 * scene.camera.h, h1 = 115./300 * scene.camera.h, w0 = 203./400 * scene.camera.w, w1 = 226./400 * scene.camera.w;
    // Canvas *subcanvas = scene.ray_trace(h0, h1, w0, w1);
    // canvas.set_submatrix(*subcanvas, h0, w0);
    // canvas.write_png(outputFile);

    return 0;
}