#pragma once
#include"Object.hpp"
#include"OBJ_Loader.hpp"
#include"BVH.hpp"
class Triangle : public Object {
public:
	Triangle(Eigen::Vector3f e1, Eigen::Vector3f e2, bool Lumin){
		edge1 = e1;
		edge2 = e2; 
		isLumin = Lumin;

	}
	Eigen::Vector4f vertex[3];
	Eigen::Vector3f normal[3];
	Eigen::Vector3f color[3];
	Eigen::Vector2f texture[3];
	void setVertex(int ind, Eigen::Vector4f v) {
		vertex[ind] = v;
	}
	void setNormal(int ind, Eigen::Vector3f n) {
		normal[ind] = n;
	}
	void setTextureCoor(int ind, Eigen::Vector2f texCoor) {
		texture[ind] = texCoor;
	}
	bool isLight() { return isLumin; }
	BoundingBox getBoundingBox() { 
		Eigen::Vector3f v0 = Eigen::Vector3f(vertex[0].x(), vertex[0].y(), vertex[0].z());
		Eigen::Vector3f v1 = Eigen::Vector3f(vertex[1].x(), vertex[1].y(), vertex[1].z());
		Eigen::Vector3f v2 = Eigen::Vector3f(vertex[2].x(), vertex[2].y(), vertex[2].z());
		return BoundingBox(v0,v0).Union(BoundingBox(v1, v1).Union(BoundingBox(v2, v2)));
	}

	Intersection getIntersection(Eigen::Vector3f ori, Eigen::Vector3f dir) {
		Intersection inter;
		Eigen::Vector3f E1 = edge1;
		Eigen::Vector3f E2 = edge2;
		Eigen::Vector3f S = ori - Eigen::Vector3f(vertex[0].x(), vertex[0].y(), vertex[0].z());
		Eigen::Vector3f S1 = dir.cross(E2);
		Eigen::Vector3f S2 = S.cross(E1);
		if (fabs(S1.dot(E1)) < 0.01)
			return inter;
		float deno = 1.0f / S1.dot(E1);
		float tnear = deno * S2.dot(E2);
		float u = deno * S1.dot(S);
		float v = deno * S2.dot(dir);

		if (tnear > 0 && u > 0 && v > 0 && (1 - u - v) > 0) {
			inter.hitHappened = true;
			inter.pos = ori + tnear * dir;
			inter.distance = tnear;
			inter.obj = this;
			
		}
		return inter;
	}
private:
	Eigen::Vector3f edge1, edge2, Vmax, Vmin;
	bool isLumin;
};