#pragma once
#include"Object.hpp"
#include"Intersection.hpp"

struct BVHnode {
public:
	BVHnode* left;
	BVHnode* right;
	BoundingBox aabb;
	Object* obj;
	BVHnode() {
		left = nullptr;
		right = nullptr;
		aabb = BoundingBox();
		obj = nullptr;
	}
};

class BVH {
public:
	BVHnode* root;
	BVH() {}
	BVH(std::vector<Object*> objList){
		root = recursivelyBuildBVH(objList);
	}
	BVHnode* recursivelyBuildBVH(std::vector<Object*> objList) {
		BVHnode* node = new BVHnode();

		if (objList.size() == 1) {
			node->left = nullptr;
			node->right = nullptr;
			node->obj = objList[0];
			node->aabb = objList[0]->getBoundingBox();
			return node;
		}
		else if (objList.size() == 2) {
			BVHnode* leftNode = new BVHnode();
			BVHnode* rightNode = new BVHnode();
			leftNode->left = nullptr;
			leftNode->right = nullptr;
			leftNode->obj = objList[0];
			leftNode->aabb = objList[0]->getBoundingBox();

			rightNode->left = nullptr;
			rightNode->right = nullptr;
			rightNode->obj = objList[1];
			rightNode->aabb = objList[1]->getBoundingBox();

			node->left = leftNode;
			node->right = rightNode;
			node->aabb = leftNode->aabb.Union(rightNode->aabb);

			return node;
		}
		else {
			BoundingBox box = BoundingBox();

			for (auto obj : objList) {
				box = box.Union(obj->getBoundingBox());
			}
			Eigen::Vector3f dis = box.distance();
			if (dis.x() >= dis.y() && dis.x() >= dis.z()) {
				std::sort(objList.begin(), objList.end(), [](auto obj1, auto obj2) {
					return obj1->getBoundingBox().Center().x() > obj2->getBoundingBox().Center().x();
					});

			}
			else if (dis.y() >= dis.x() && dis.y() >= dis.z()) {
				std::sort(objList.begin(), objList.end(), [](auto obj1, auto obj2) {
					return obj1->getBoundingBox().Center().y() > obj2->getBoundingBox().Center().y();
					});
			}
			else if (dis.z() >= dis.x() && dis.z() >= dis.y()) {
				std::sort(objList.begin(), objList.end(), [](auto obj1, auto obj2) {
					return obj1->getBoundingBox().Center().z() > obj2->getBoundingBox().Center().z();
					});
			}

			auto start = objList.begin();
			auto end = objList.end();
			auto mid = objList.begin() + objList.size() / 2;
			auto leftObjList = std::vector<Object*>(start, mid);
			auto rightObjList = std::vector<Object*>(mid, end);

			node->left = recursivelyBuildBVH(leftObjList);
			node->right = recursivelyBuildBVH(rightObjList);

			node->aabb = node->left->aabb.Union(node->right->aabb);
		}
		return node;
	}
	Intersection getIntersection(Eigen::Vector3f ori, Eigen::Vector3f dir) {
		Intersection inter;
		if (!root)
			return inter;
		inter = intersect(root, ori, dir);
		return inter;
	}
	Intersection intersect(BVHnode* node, Eigen::Vector3f ori, Eigen::Vector3f dir) {
		Intersection inter;
		if (!node->aabb.intersect(ori, dir)) return inter;
		if (node->left == nullptr && node->right == nullptr) return node->obj->getIntersection(ori, dir);
		Intersection hit1 = intersect(node->left, ori, dir);
		Intersection hit2 = intersect(node->right, ori, dir);
		if (hit1.distance < hit2.distance) return hit1;
		else return hit2;
	}
};