#pragma once
#include"Object.hpp"

struct Intesection {
	Intesection() {
		distance = std::numeric_limits<float>::infinity();
		hitHappened = false;

		obj = nullptr;
	}
	float distance;
	bool hitHappened;
	
	Object* obj;
};