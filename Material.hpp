#pragma once
#include<Eigen/Eigen>
#include"function.hpp"

enum MaterialType {DIFFUSE};
const float PI = 3.141592653589793f;

class Material {
public:
	Material(MaterialType t, Eigen::Vector3f e) {
		mType = t;
		emit = e;
	}
	MaterialType mType;
	Eigen::Vector3f abd;
	Eigen::Vector3f emit;
	bool luminuous() {
		if (emit.norm() > 0.001)
			return true;
		else return false;
	}
	Eigen::Vector3f evalDiffuse(Eigen::Vector3f wi, Eigen::Vector3f wo, Eigen::Vector3f N) {
		switch (mType)
		{
		case DIFFUSE:
			if (wo.dot(N) > 0.0f) return abd / PI;
			else return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
			break;
		default:
			break;
		}
	}

	Eigen::Vector3f eval(Eigen::Vector3f wi, Eigen::Vector3f wo, Eigen::Vector3f N) {
		switch (mType)
		{
		case DIFFUSE:
			return evalDiffuse(wi, wo, N);
			break;

		default:
			break;
		}
	
	}

	Eigen::Vector3f sampleSemiSH() {
		float rand1 = getRandomNum();
		float rand2 = getRandomNum();

		float phi = 2 * PI * rand2;
		float z = std::sqrt(1.0f - rand1 * rand1);
		float x = rand1 * cos(phi);
		float y = rand1 * sin(phi);
		return Eigen::Vector3f(x, y, z);
	}

	Eigen::Vector3f localToWorld(Eigen::Vector3f localDir, Eigen::Vector3f N) {
		Eigen::Vector3f T, B;
		if ((N.x() * N.x() + N.z() * N.z()) > 0) {
			float inv = 1.0f / (N.x() * N.x() + N.z() * N.z());
			T = inv * Eigen::Vector3f(N.z(), 0.0f, -N.x());
			B = N.cross(T);
		}
		else {
			float inv = 1.0f / (N.y() * N.y() + N.z() * N.z());
			T = inv * Eigen::Vector3f(0.0f, N.z(), -N.y());
			B = N.cross(T);
		}

		return localDir.x() * T + localDir.y() * B + localDir.z() * N;
	}

	Eigen::Vector3f sampleDir(Eigen::Vector3f N) {
		Eigen::Vector3f localDir = sampleSemiSH();

		return localToWorld(localDir, N);
	}

	float uniformPDF(Eigen::Vector3f wi, Eigen::Vector3f wo, Eigen::Vector3f N) {
		if (wo.dot(N) > 0)
			return 1.0f / (2.0f * PI);
		else
			return 1.0f;
	}
};