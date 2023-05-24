#pragma once
#include"Object.hpp"

struct Intersection {
	Intersection() {
		distance = std::numeric_limits<float>::infinity();
		hitHappened = false;
		pos = Eigen::Vector3f(std::numeric_limits<float>::infinity(),
			std::numeric_limits<float>::infinity(),
			std::numeric_limits<float>::infinity());
		obj = nullptr;
	}
	float distance;
	bool hitHappened;
	Eigen::Vector3f pos;
	Object* obj;
};