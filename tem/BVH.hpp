#pragma once
#include"Object.hpp"
#include"Intesection.hpp"
struct BVHnode {
	BVHnode* left;
	BVHnode* right;
	BoundingBox* aabb;
	Object* obj;
};

class BVH {
public:
	BVHnode* root;
	BVH(BVHnode* r): root(r){}
	Intersection getIntersection(Eigen::Vector3f ori, Eigen::Vector3f dir) {
		Intersection inter = intersect(root, ori, dir);
		return inter;
	}
	Intersection intersect(BVHnode* node, Eigen::Vector3f ori, Eigen::Vector3f dir) {
		Intersection inter;
		if (!root->aabb.intersect(ori, dir)) return inter;
		//if (node->left == nullptr && node->right == nullptr) return node->object->getIntersection(ori, dir);
		Intersection hit1 = intersect(node->left, ori, dir);
		Intersection hit2 = intersect(node->right, ori, dir);
		if (hit1.distance < hit2.distance) return hit1;
		else return hit2;
	}
};