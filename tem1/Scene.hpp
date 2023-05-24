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
	
private:
	std::vector<Object*> objList;
	BVHnode* root;
	BVH* sceneBVH;
};