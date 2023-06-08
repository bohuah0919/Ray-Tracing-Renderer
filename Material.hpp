#pragma once
#include<Eigen/Eigen>
#include"function.hpp"

enum MaterialType {DIFFUSE, MICROFACET, SPECULAR};
const float PI = 3.141592653589793f;

class Material {
public:
	Material(MaterialType t, Eigen::Vector3f e) {
		mType = t;
		emit = e;
	}

	MaterialType mType;
	Eigen::Vector3f albedo;
	float roughness;
	Eigen::Vector3f emit;
	float ior;

	bool luminuous() {
		if (emit.norm() > 0.00001f)
			return true;
		else return false;
	}

	Eigen::Vector3f evalDiffuse(Eigen::Vector3f wo, Eigen::Vector3f wi, Eigen::Vector3f N) {
		if (wi.dot(N) > 0.0f) return albedo / PI;
		else return Eigen::Vector3f(0.0f, 0.0f, 0.0f);

	}

	Eigen::Vector3f evalMicrofacet(Eigen::Vector3f wo, Eigen::Vector3f wi, Eigen::Vector3f N) {
		if ((wi.dot(N) > 0.0f)) {
			float OdotN = std::max(wo.dot(N), 0.0f);
			float IdotN = std::max(wi.dot(N), 0.0f);
			Eigen::Vector3f H = (wi + wo).normalized();
			float F = fresnelSchlick(wo, H);
			float D = GGX(H, N);
			float G = GSmith(wo, wi, N);
			float specular = F * G * D / (4 * OdotN * IdotN);
			Eigen::Vector3f diffuse = (1 - F) * albedo / PI;
			Eigen::Vector3f fr = Eigen::Vector3f(specular, specular, specular) + diffuse;
			return fr;
		}
		else return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
	}

	Eigen::Vector3f evalSpecular(Eigen::Vector3f wo, Eigen::Vector3f wi, Eigen::Vector3f N) {
		if ((wi.dot(N) > 0.0f)) {
			float F = fresnelSchlick(wo, N);
			Eigen::Vector3f brdf = Eigen::Vector3f(F, F, F);
			return brdf;
		}
		else return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
	}

	Eigen::Vector3f eval(Eigen::Vector3f wo, Eigen::Vector3f wi, Eigen::Vector3f N) {
		switch (mType)
		{
		case DIFFUSE:
			return evalDiffuse(wo, wi, N);
			break;
		case MICROFACET:
			return evalMicrofacet(wo, wi, N);
			break;
		case SPECULAR:
			return evalSpecular(wo, wi, N);
			break;
		}
	}

	Eigen::Vector3f sampleSemiSH() {
		float rand1 = getRandomFloat();
		float rand2 = getRandomFloat();

		float phi = 2 * PI * rand2;
		float z = std::fabs(1.0f - 2.0f * rand1);
		float r = std::sqrt(1.0f - z * z);
		float x = r * cos(phi);
		float y = r * sin(phi);
		return Eigen::Vector3f(x, y, z);
	}

	Eigen::Vector3f localToWorld(Eigen::Vector3f localDir, Eigen::Vector3f N) {
		Eigen::Vector3f T, B;
		if ((N.x() * N.x() + N.z() * N.z()) > 0) {
			float inv = 1.0f / std::sqrt(N.x() * N.x() + N.z() * N.z());
			T = inv * Eigen::Vector3f(N.z(), 0.0f, -N.x());
			B = N.cross(T);
		}
		else {
			float inv = 1.0f / std::sqrt(N.y() * N.y() + N.z() * N.z());
			T = inv * Eigen::Vector3f(0.0f, N.z(), -N.y());
			B = N.cross(T);
		}

		return localDir.x() * T + localDir.y() * B + localDir.z() * N;
	}

	Eigen::Vector3f sampleDir(Eigen::Vector3f wo, Eigen::Vector3f N) {
		Eigen::Vector3f localDir;
		switch (mType)
		{
		case DIFFUSE:
			localDir = sampleSemiSH();
			return localToWorld(localDir, N);
			break;
		case MICROFACET:
			localDir = sampleSemiSH();
			return localToWorld(localDir, N);
			break;
		case SPECULAR:
			return reflect(wo, N);
			break;
		}
	}

	Eigen::Vector3f reflect(Eigen::Vector3f wi, Eigen::Vector3f N) {
		return 2.0f * wi.dot(N) * N - wi;
	}

	float PDF(Eigen::Vector3f wo, Eigen::Vector3f wi, Eigen::Vector3f N) {
		switch (mType)
		{
		case DIFFUSE:
			if (wi.dot(N) > 0)
				return 0.5f / PI;
			else
				return 1.0f;
			break;
		case MICROFACET:
			if (wi.dot(N) > 0)
				return 0.5f / PI;
			else
				return 1.0f;
			break;
		case SPECULAR:
			return 1.0f;
			break;
		}
	}

	float fresnelSchlick( Eigen::Vector3f wo, Eigen::Vector3f N) {
		float cos = wo.dot(N);
		if (cos < 0) return 0.0f;
		float n1 = 1.0f;
		float n2 = ior;
		float R0 = std::pow((n1 - n2) / (n1 + n2), 2.0f);
		return R0 + (1 - R0) * std::pow((1 - cos), 5.0f);
	}

	float GGX(Eigen::Vector3f H, Eigen::Vector3f N) {
		float alpha = roughness * roughness;
		float HdotN = std::max(H.dot(N), 0.0f);
		float inv = 1.0f / (PI * std::pow(HdotN * HdotN * (alpha * alpha - 1.0f) + 1.0f, 2.0f));
		return alpha * alpha * inv;
	}

	float GSchlick(Eigen::Vector3f V, Eigen::Vector3f N) {
		float k = roughness * roughness / 2.0f;
		float VdotN = std::max(V.dot(N), 0.0f);
		return VdotN / (VdotN * (1.0f - k) + k);
	}

	float GSmith(Eigen::Vector3f wo, Eigen::Vector3f wi, Eigen::Vector3f N) {
		return GSchlick(wo, N) * GSchlick(wi, N);
	}
};