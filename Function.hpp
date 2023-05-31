#pragma once
#include <random>

float getRandomNum() {
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution <float> dis(0.0f, 1.0f);
	return dis(gen);
}