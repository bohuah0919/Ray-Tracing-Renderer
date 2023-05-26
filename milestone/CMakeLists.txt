cmake_minimum_required(VERSION 3.10) 

project(RayTracing)
find_package(Eigen3 3.3 REQUIRED)
set(CMAKE_CXX_STANDARD 17) 

add_executable(RayTracing main.cpp Object.hpp OBJ_Loader.hpp Triangle.hpp MeshTriangle.hpp BoundingBox.hpp BVH.hpp Intersection.hpp) 
target_link_libraries(RayTracing Eigen3::Eigen)