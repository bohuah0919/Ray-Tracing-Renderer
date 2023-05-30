#pragma once
#include<Eigen/Eigen>
#include"BoundingBox.hpp"
#include"Intersection.hpp"
#include <random>

inline float getRandomNum() {
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution <float> dis(0.0f, 1.0f);
	return dis(gen);
}

class Object {
public:
	Object() {}
	virtual ~Object() {}
	virtual bool isLight() = 0;
	virtual BoundingBox getBoundingBox() = 0;
	virtual Intersection getIntersection(Eigen::Vector3f ori, Eigen::Vector3f dir) = 0;
	virtual float getArea() = 0;
	virtual void sample(Intersection& inter, float& pdf) = 0;
};