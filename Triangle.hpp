#pragma once
#include"Object.hpp"
#include"OBJ_Loader.hpp"
#include"BVH.hpp"
#include"Material.hpp"

class Triangle : public Object {
public:
	Triangle(Eigen::Vector3f e1, Eigen::Vector3f e2, Material* m){
		edge1 = e1;
		edge2 = e2; 
		area = (e1.cross(e2)).norm()*0.5f;
		material = m;
	}
	Eigen::Vector4f vertex[3];
	Eigen::Vector3f normal[3];
	Eigen::Vector2f textureCoor[3];
	void setVertex(int ind, Eigen::Vector4f v) {
		vertex[ind] = v;
	}
	void setNormal(int ind, Eigen::Vector3f n) {
		normal[ind] = n;
	}
	void setTextureCoor(int ind, Eigen::Vector2f texCoor) {
		textureCoor[ind] = texCoor;
	}
	bool isLight() { return material->luminuous(); }
	float getArea() { return area; }
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
		float b1 = deno * S1.dot(S);
		float b2 = deno * S2.dot(dir);

		if (tnear > 0 && b1 > 0 && b2 > 0 && (1 - b1 - b2) > 0) {
			inter.hitHappened = true;
			inter.pos = ori + tnear * dir;
			inter.distance = tnear;
			inter.obj = this;
			inter.normal = (1 - b1 - b2) * normal[0] + b1 * normal[1] + b2 * normal[2];
			inter.material = this->material;
			//std::cout << normal[0] << "\n\n";
		}
		return inter;
	}

	void sample(Intersection& inter, float& pdf) {
		float rand1 = getRandomNum();
		float rand2 = (1.0f - rand1) * getRandomNum();

		Eigen::Vector3f v0 = Eigen::Vector3f(vertex[0].x(), vertex[0].y(), vertex[0].z());
		Eigen::Vector3f v1 = Eigen::Vector3f(vertex[1].x(), vertex[1].y(), vertex[1].z());
		Eigen::Vector3f v2 = Eigen::Vector3f(vertex[2].x(), vertex[2].y(), vertex[2].z());

		inter.material = this->material;
		inter.pos = v0 * (1 - rand1 - rand2) + v1 * rand1 + v2 * rand2;
		inter.normal = (normal[0] * (1 - rand1 - rand2) + normal[1] * rand1 + normal[2] * rand2).normalized();
	}
private:
	Material* material;
	Eigen::Vector3f edge1, edge2, Vmax, Vmin;
	bool isLumin;
	float area;
};