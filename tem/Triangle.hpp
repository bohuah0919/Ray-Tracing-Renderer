#pragma once
#include"Object.hpp"
#include"OBJ_Loader.hpp"

class Triangle : public Object {
public:
	Triangle(Eigen::Vector3f e1, Eigen::Vector3f e2, bool Lumin){
		edge1 = e1;
		edge2 = e2; 
		isLumin = Lumin;

		Vmax = -Eigen::Vector3f(std::numeric_limits<float>::infinity(),
			std::numeric_limits<float>::infinity(),
			std::numeric_limits<float>::infinity());
		Vmin = Eigen::Vector3f(std::numeric_limits<float>::infinity(),
			std::numeric_limits<float>::infinity(),
			std::numeric_limits<float>::infinity());


	}
	Eigen::Vector4f vertex[3];
	Eigen::Vector3f normal[3];
	Eigen::Vector3f color[3];
	Eigen::Vector2f texture[3];
	void setVertex(int ind, Eigen::Vector4f v) {
		vertex[ind] = v;
		Vmax(0) = std::max(Vmax.x(), v.x());
		Vmax(1) = std::max(Vmax.y(), v.y());
		Vmax(2) = std::max(Vmax.z(), v.z());

		Vmin(0) = std::min(Vmin.x(), v.x());
		Vmin(1) = std::min(Vmin.y(), v.y());
		Vmin(2) = std::min(Vmin.z(), v.z());
	}
	void setNormal(int ind, Eigen::Vector3f n) {
		normal[ind] = n;
	}
	void setTextureCoor(int ind, Eigen::Vector2f texCoor) {
		texture[ind] = texCoor;
	}
	bool isLight() { return isLumin; }

	void buildBoundingBox() { aabb = BoundingBox(Vmax, Vmin); }
	BoundingBox getBoundingBox() { return BoundingBox(); }

	Intersection getIntersection(Eigen::Vector3f ori, Eigen::Vector3f dir) {
		Eigen::Vector3f E1 = edge1;
		Eigen::Vector3f E2 = edge2;
		Eigen::Vector3f S = orig - Eigen::Vector3f(vertex[0].x(), vertex[0].y(), vertex[0].z());
		Eigen::Vector3f S1 = dir.cross3(E2);
		Eigen::Vector3f S2 = S.cross3(E1);
		float deno = 1 / S1.dotProduct(E1);
		float tnear = deno * S2.dotProduct(E2);
		float u = deno * S1.dotProduct(S);
		float v = deno * dotProduct(S2, dir);
		if (tnear > 0 && u > 0 && v > 0 && (1 - u - v) > 0) return true;
		return false;
	}
private:
	Eigen::Vector3f edge1, edge2, Vmax, Vmin;
	BoundingBox aabb;
	bool isLumin;
};