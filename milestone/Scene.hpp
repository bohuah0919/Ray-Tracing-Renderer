#pragma once
#include"Object.hpp"
#include"BVH.hpp"
class Scene {
public:
	float width, height;
	Scene(float w, float h): width(w), height(h){}
	void addObj(Object* obj) {objList.push_back(obj);}
	void buildBVH() { 
		sceneBVH = new BVH(objList);
	}
	Intersection getIntersection(Eigen::Vector3f ori, Eigen::Vector3f dir) {
		Intersection inter;
		if (sceneBVH) {
			inter = sceneBVH->getIntersection(ori, dir);
		}
		return inter;
	}
	
private:
	std::vector<Object*> objList;
	BVH* sceneBVH;
};