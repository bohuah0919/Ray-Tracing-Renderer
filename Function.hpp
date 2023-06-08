#pragma once
#include <random>

float getRandomFloat() {
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::uniform_real_distribution <float> dis(0.0f, 1.0f);
	return dis(gen);
}
bool solvingQuadraticEquation(float a, float b, float c, float& x1, float& x2) {
	float delta = b * b - 4.0f * a * c;
	if (delta < 0) return false;
	float inv = 0.5f / a;
	x1 = (-b - std::sqrt(delta)) * inv;
	x2 = (-b + std::sqrt(delta)) * inv;
	return true;
}