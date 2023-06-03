#include <iostream>
#include "MeshTriangle.hpp"
#include "Scene.hpp"
#include <algorithm>
#include <fstream>
#include <thread>
#include <mutex>
void Render(Scene& scene)
{
    std::vector<Eigen::Vector3f> framebuffer(scene.width * scene.height);
    std::fill(framebuffer.begin(), framebuffer.end(), Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    float scale = tan((scene.fov * 0.5) * PI / 180.0);
    float imageAspectRatio = scene.width / (float)scene.height;
    Eigen::Vector3f eye_pos(278.0f, 273.0f, -800.0f);
    std::mutex mtx;
    int process = 0;

    int sampleNum = 32;
    int threadNum = 4;
    std::vector<std::thread> threads(threadNum);
    int threadSize = scene.height / threadNum;
    auto generateRay = [&](int threadBegin, int threadEnd) {
        for (int j = threadBegin; j < threadEnd; ++j) {
            for (int i = 0; i < scene.width; ++i) {

                float x = (2.0f * (i + 0.5f) / (float)scene.width - 1) *
                    imageAspectRatio * scale;
                float y = (1.0f - 2.0f * (j + 0.5f) / (float)scene.height) * scale;

                Eigen::Vector3f dir = Eigen::Vector3f(-x, y, 1.0f).normalized();
                for (int k = 0; k < sampleNum; k++) {
                    framebuffer[j * scene.height + i] += scene.castRay(eye_pos, dir) / sampleNum;
                }
            }
            mtx.lock();
            process++;
            std::cout << process * 100 / scene.height << "% \n\n";
            mtx.unlock();
        }
    };
    for (int t = 0; t < threadNum; t++) {
        threads[t] = std::thread(generateRay, t * threadSize, (t + 1) * threadSize);
    }
    for (int t = 0; t < threadNum; t++) {
        threads[t].join();
    }
 
    
    FILE* fp = fopen("D:/cornellbox.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(std::max(0.0f, std::min(1.0f, framebuffer[i].x())), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(std::max(0.0f, std::min(1.0f, framebuffer[i].y())), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(std::max(0.0f, std::min(1.0f, framebuffer[i].z())), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);
}


int main(int argc, char** argv) {
    Material* red = new Material(DIFFUSE, Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    red->abd = Eigen::Vector3f(0.63f, 0.065f, 0.05f);
    Material* green = new Material(DIFFUSE, Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    green->abd = Eigen::Vector3f(0.14f, 0.45f, 0.091f);
    Material* white = new Material(DIFFUSE, Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    white->abd = Eigen::Vector3f(0.725f, 0.71f, 0.68f);
    Material* lumin = new Material(DIFFUSE, (8.0f * Eigen::Vector3f(0.747f + 0.058f, 0.747f + 0.258f, 0.747f) + 15.6f * Eigen::Vector3f(0.740f + 0.287f, 0.740f + 0.160f, 0.740f) + 18.4f * Eigen::Vector3f(0.737f + 0.642f, 0.737f + 0.159f, 0.737f)));
    lumin->abd = Eigen::Vector3f(0.65f, 0.65f, 0.65f);

    MeshTriangle floor("D:/assignment7/models/cornellbox/floor.obj",false,false, white);
    MeshTriangle left("D:/assignment7/models/cornellbox/left.obj", false, false, red);
    MeshTriangle right("D:/assignment7/models/cornellbox/right.obj", false, false, green);
    MeshTriangle shortbox("D:/assignment7/models/cornellbox/shortbox.obj", false, false, white);
    MeshTriangle tallbox("D:/assignment7/models/cornellbox/tallbox.obj", false, false, white);
    MeshTriangle light("D:/assignment7/models/cornellbox/light.obj", false, false, lumin);

    Scene scene(768, 768);
    scene.addObj(&floor);
    scene.addObj(&left);
    scene.addObj(&right);
    scene.addObj(&shortbox);
    scene.addObj(&tallbox);
    scene.addObj(&light);

    scene.buildBVH();

    Render(scene);
    return 0;
}