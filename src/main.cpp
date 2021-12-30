#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <algorithm>
#include "Vectors.h"
#include "Object.h"
#include "Scene.h"
#include "Bezier.h" // TODO: delete this


void draw_subcanvas(int num_workers, int worker, Scene *scene, Canvas *canvas, int samp, int seed) {
    int delta_h = (scene->camera.h + num_workers - 1) / num_workers;
    int h0 = worker * delta_h, h1 = std::min(scene->camera.h, (worker + 1) * delta_h);
    if (h0 >= h1) return;
    Canvas *subcanvas = scene->ray_trace(h0, h1, 0, scene->camera.w, samp, seed);
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

    int dot_pos = outputFile.find_last_of('.');
    std::string output_base_name = outputFile.substr(0, dot_pos), output_ext_name = outputFile.substr(dot_pos + 1);
    if (output_ext_name != "png") {
        std::cout << "Error: output must be .png file." << std::endl;
        return 1;
    }
    std::string logFile = output_base_name + "_log.txt";
    std::ifstream log_file;
    int done_iters = 0;
    log_file.open(logFile, std::ios::in);
    if(!log_file) {
        std::cout << "Log file not found. Starting rendering from scratch..." << std::endl;
    }
    else {
        log_file >> done_iters;
        while (true){
            std::cout << "Found log file " << logFile << ". Continue rendering from iteration " << done_iters << "? (y/n)";
            std::cout.flush();
            std::string tmp;
            std::cin >> tmp;
            if (tmp[0] == 'n') {
                done_iters = 0;
                std::cout << "Log file discarded. Starting rendering from scratch..." << std::endl;
            }
            else if (tmp[0] != 'y') continue;
            break;
        }
        log_file.close();
    }
    
    Scene scene(inputFile);
    Canvas output_canvas(scene.camera.w, scene.camera.h);
    if (done_iters) output_canvas.read_png(outputFile);

    for (int iter = done_iters, last_save = done_iters; iter < scene.samp; iter = iter + scene.save_interval){
        Canvas canvas(scene.camera.w, scene.camera.h);
        int num_workers = 16;
        std::thread *threads[num_workers];
        for(int i = 0; i < num_workers; i++) {
            threads[i] = new std::thread(draw_subcanvas, num_workers, i, &scene, &canvas, scene.save_interval, iter);
        }
        for(int i = 0; i < num_workers; i++) {
            threads[i]->join();
            delete threads[i];
        }

        for (int y = 0; y < scene.camera.h; y++)
            for (int x = 0; x < scene.camera.w; x++)
                output_canvas[y][x] = (output_canvas[y][x] * last_save + canvas[y][x] * (iter + 1 - last_save)) * (1./(iter + 1));
        last_save = iter + 1;
        output_canvas.write_png(outputFile);

        std::ofstream log_file;
        log_file.open(logFile, std::ios::out);
        log_file << iter + 1 << std::endl;
        log_file.close();

        std::cout << "Saved " << outputFile << " at iteration " << iter + 1 << "/" << scene.samp << "." << std::endl;
    }
    
    // int h0 = 0 * scene.camera.h, h1 = 0.1 * scene.camera.h, w0 = 0.9 * scene.camera.w, w1 = 1 * scene.camera.w;
    // Canvas *subcanvas = scene.ray_trace(h0, h1, w0, w1, 50, 0);
    // output_canvas.set_submatrix(*subcanvas, h0, w0);
    // output_canvas.write_png(outputFile);

    return 0;
}