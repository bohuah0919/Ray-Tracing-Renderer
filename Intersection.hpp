#pragma once
#include"Object.hpp"
#include"Material.hpp"
class Object;
struct Intersection {
public:
	Intersection() {
		distance = std::numeric_limits<float>::infinity();
		hitHappened = false;
		pos = Eigen::Vector3f(std::numeric_limits<float>::infinity(),
			std::numeric_limits<float>::infinity(),
			std::numeric_limits<float>::infinity());
		obj = nullptr;
		material = nullptr;
	}
	Object* obj;
	float distance;
	bool hitHappened;
	Eigen::Vector3f pos;
	Eigen::Vector3f normal;
	//Eigen::Vector3f emit;
	//Eigen::Vector2f uv;
	Material* material;
};