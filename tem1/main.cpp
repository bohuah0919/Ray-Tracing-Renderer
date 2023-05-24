#include <iostream>
#include"MeshTriangle.hpp"
#include"Scene.hpp"
constexpr double MY_PI = 3.1415926;


int main(int argc, char** argv) {


    MeshTriangle floor("D:/assignment7/models/cornellbox/floor.obj",false,false,false);
    MeshTriangle left("D:/assignment7/models/cornellbox/left.obj", false, false, false);
    MeshTriangle right("D:/assignment7/models/cornellbox/right.obj", false, false, false);
    MeshTriangle shortbox("D:/assignment7/models/cornellbox/shortbox.obj", false, false, false);
    MeshTriangle tallbox("D:/assignment7/models/cornellbox/tallbox.obj", false, false, false);
    MeshTriangle light("D:/assignment7/models/cornellbox/light.obj", false, false, true);

    Scene scene(700, 700);
    scene.addObj(&floor);
    scene.addObj(&right);
    scene.addObj(&shortbox);
    scene.addObj(&tallbox);
    scene.addObj(&light);
    scene.buildBVH();

    std::cout << floor.getBoundingBox().Vmax << "\n\n";
    std::cout << floor.getBoundingBox().Vmin;
    return 0;
}