#include <iostream>
#include"MeshTriangle.hpp"
#include"Scene.hpp"
constexpr double MY_PI = 3.1415926;

void printAABB(BVHnode* node) {
    std::cout << node->aabb.Vmax << "\n";
    std::cout << node->aabb.Vmin << "\n\n";

    if (node->obj) {
        return;
    }
        
    printAABB(node->left);
    printAABB(node->right);
}
int main(int argc, char** argv) {


    MeshTriangle floor("D:/assignment7/models/cornellbox/floor.obj",false,false,false);
    MeshTriangle left("D:/assignment7/models/cornellbox/left.obj", false, false, false);
    MeshTriangle right("D:/assignment7/models/cornellbox/right.obj", false, false, false);
    MeshTriangle shortbox("D:/assignment7/models/cornellbox/shortbox.obj", false, false, false);
    MeshTriangle tallbox("D:/assignment7/models/cornellbox/tallbox.obj", false, false, false);
    MeshTriangle light("D:/assignment7/models/cornellbox/light.obj", false, false, true);

    Scene scene(700, 700);
    scene.addObj(&floor);
    scene.addObj(&left);
    scene.addObj(&right);
    scene.addObj(&shortbox);
    scene.addObj(&tallbox);
    scene.addObj(&light);

    scene.buildBVH();

    std::cout << shortbox.getBoundingBox().Vmax << "\n";
    std::cout << shortbox.getBoundingBox().Vmin << "\n\n";

    //printAABB(scene.sceneBVH->root);

    Intersection inter = scene.getIntersection(Eigen::Vector3f(278, 273, -800), Eigen::Vector3f(0, 0, 1));

    std::cout << floor.triangleList[5]->getBoundingBox().Vmax << "\n";
    std::cout << floor.triangleList[5]->getBoundingBox().Vmin << "\n\n";
    std::cout << floor.getBoundingBox().intersect(Eigen::Vector3f(278, 273, -800), Eigen::Vector3f(0, 0, 1)) << "\n\n";
    std::cout << inter.obj->getBoundingBox().Vmax << "\n\n";

    return 0;
}