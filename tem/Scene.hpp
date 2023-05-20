#pragma once
#include"Object.hpp"
#include"BVH.hpp"
class Scene {
public:
	float width, height;
	Scene(float w, float h): width(w), height(h){}
	void addObj(Object* obj) {objList.push_back(obj);}
	void buildBVH() { 
		recursivelyBuildBVH(root, objList);
		sceneBVH = BVH(root);
	}
	void recursivelyBuildBVH(BVHnode* node, std::vector<Object*> objList) {
		if (objList.size() == 1) {
			node->left = nullptr;
			node->right = nullptr;
			node->obj = objList[0];
			node->aabb = objList[0].getBoundingBox();
			return;
		}
		else if (objList.size() == 2) {
			BVHnode* leftNode;
			BVHnode* rightNode;
			leftNode->left = nullptr;
			leftNode->right = nullptr;
			leftNode->obj = objList[0];

			rightNode->left = nullptr;
			rightNode->right = nullptr;
			rightNode->obj = objList[1];

			node->left = leftNode;
			node->right = rightNode;
			node->aabb = objList[0].getBoundingBox().Union(objList[1].getBoundingBox());
			return;
		}
		else {
			BoundingBox box = BoundingBox();
			
			for (auto obj : objList) {
				box = box.Union(obj.getBoundingBox());
			}
			Eigen:Vector3f dis = box.distance();
			if (dis.x() >= dis.y() && dis.x() >= dis.z()) {
				std::sort(objList.begin(), objList.end(), [](auto obj1, auto obj2) {
					return obj1.getBoundingBox().Center().x() > obj2.getBoundingBox().Center().x();
					})
			
			}
			else if (dis.y() >= dis.x() && diy.y() >= dis.z()) {
				std::sort(objList.begin(), objList.end(), [](auto obj1, auto obj2) {
					return obj1.getBoundingBox().Center().y() > obj2.getBoundingBox().Center().y();
					})
			}
			else if (dis.z() >= dis.x() && dis.z() >= dis.y()) {
				std::sort(objList.begin(), objList.end(), [](auto obj1, auto obj2) {
					return obj1.getBoundingBox().Center().z() > obj2.getBoundingBox().Center().z();
					})
			}

			auto start = objList.begin();
			auto end = objList.end();
			auto mid = objList.begin() + objList.size()/2;
			auto leftObjList = std::vector<Object*>(start, mid);
			auto rightObjList = std::vector<Object*>(mid, end);

			BVHnode* leftNode;
			BVHnode* rightNode;

			node->left = leftNode;
			node->right = rightNode;
			recursivelyBuildBVH(leftNode, leftObjList);
			recursivelyBuildBVH(rightNode, rightObjList);
			node->aabb = node->left->aabb.Union(node->right->aabb);
		}
	}
private:
	std::vector<Object*> objList;
	BVHnode* root;
	BVH sceneBVH;
};