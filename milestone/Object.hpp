#pragma once
#include<Eigen/Eigen>
#include"BoundingBox.hpp"
#include"Intersection.hpp"

class Object {
public:
	Object() {}
	virtual ~Object() {}
	virtual bool isLight() = 0;
	virtual BoundingBox getBoundingBox() = 0;
	virtual Intersection getIntersection(Eigen::Vector3f ori, Eigen::Vector3f dir) = 0;
};