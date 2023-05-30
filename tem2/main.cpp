#include <iostream>
#include"MeshTriangle.hpp"
#include"Scene.hpp"

const float PI = 3.141592653589793f;

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
    Material* red = new Material(DIFFUSE, Eigen::Vector3f(0.0f));
    red->abd = Eigen::Vector3f(0.63f, 0.065f, 0.05f);
    Material* green = new Material(DIFFUSE, Eigen::Vector3f(0.0f));
    green->abd = Eigen::Vector3f(0.14f, 0.45f, 0.091f);
    Material* white = new Material(DIFFUSE, Eigen::Vector3f(0.0f));
    white->abd = Eigen::Vector3f(0.725f, 0.71f, 0.68f);
    Material* lumin = new Material(DIFFUSE, (8.0f * Eigen::Vector3f(0.747f + 0.058f, 0.747f + 0.258f, 0.747f) + 15.6f * Eigen::Vector3f(0.740f + 0.287f, 0.740f + 0.160f, 0.740f) + 18.4f * Eigen::Vector3f(0.737f + 0.642f, 0.737f + 0.159f, 0.737f)));
    lumin->abd = Eigen::Vector3f(0.65f);

    MeshTriangle floor("models/cornellbox/floor.obj",false,false, white);
    MeshTriangle left("models/cornellbox/left.obj", false, false, red);
    MeshTriangle right("models/cornellbox/right.obj", false, false, green);
    MeshTriangle shortbox("models/cornellbox/shortbox.obj", false, false, white);
    MeshTriangle tallbox("models/cornellbox/tallbox.obj", false, false, white);
    MeshTriangle light("models/cornellbox/light.obj", false, false, lumin);

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
    std::cout << inter.pos << "\n\n";

    return 0;
}