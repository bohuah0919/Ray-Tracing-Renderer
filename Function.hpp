#pragma once
#include <random>

float getRandomNum() {
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::uniform_real_distribution <float> dis(0.0f, 1.0f);
	return dis(gen);
}