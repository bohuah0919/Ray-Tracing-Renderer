#pragma once
#include"Object.hpp"
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
	}
	Object* obj;
	float distance;
	bool hitHappened;
	Eigen::Vector3f pos;

};