#pragma once
#include"Object.hpp"
#include"OBJ_Loader.hpp"
#include"BVH.hpp"
#include"Material.hpp"

class Sphere : public Object {
public:
	Sphere(Eigen::Vector3f c, float r, Material* m) : center(c), radius(r), area(4 * PI * r * r), material(m) {}
	bool isLight() { return material->luminuous(); }
	float getArea() { return area; }
	BoundingBox getBoundingBox() { 
		Eigen::Vector3f r = Eigen::Vector3f(radius + 0.00001f, radius + 0.00001f, radius + 0.00001f);
		Eigen::Vector3f vmax = center + r;
		Eigen::Vector3f vmin = center - r;
		return BoundingBox(vmax, vmin);
	}
	Intersection getIntersection(Eigen::Vector3f ori, Eigen::Vector3f dir) {
		Intersection inter;
		float x1,x2,tnear;
		float a = 1.0f;
		float b = 2.0f * (ori - center).dot(dir);
		float c = (ori - center).squaredNorm() - radius * radius;

		if (!solvingQuadraticEquation(a, b, c, x1, x2)) return inter;
		if (x1 < 0.0f && x2 < 0.0f) return inter;
		else if (x1 >= 0) tnear = x1;
		else tnear = x2;

		Eigen::Vector3f n = (ori + tnear * dir - center).normalized();
		if (dir.dot(n) > 0) return inter;

		if (tnear > 0.5f) {
			inter.hitHappened = true;
			inter.pos = ori + tnear * dir;
			inter.distance = tnear;
			inter.obj = this;
			inter.normal = (inter.pos - center).normalized();
			inter.material = this->material;
		}
		
		return inter;
	}
	void sample(Intersection& inter, float& pdf) {
		float rand1 = getRandomFloat();
		float rand2 = getRandomFloat();

		float phi = 2 * PI * rand2;
		float z = 1.0f - 2.0f * rand1;
		float r = std::sqrt(1.0f - z * z);
		float x = r * cos(phi);
		float y = r * sin(phi);

		Eigen::Vector3f dir = Eigen::Vector3f(x, y, z);

		inter.pos = center + radius * dir;
		inter.normal = dir;
		inter.material = this->material;

		pdf = 1.0f / area;
	}
private:
	Eigen::Vector3f center;
	float radius;
	float area;
	Material* material;
};
