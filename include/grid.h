#ifndef GRID_H
#define GRID_H

#include "math/vec.h"
#include <array>

struct Grid
{
	std::array<Vec3, 8> pos;
	std::array<Real, 8> valueOfPos;

	static const std::array<std::pair<int, int>, 12> edgeIdx = { {
		{0, 1}, {1, 2}, {2, 3}, {3, 0},
		{4, 5}, {5, 6}, {6, 7}, {7, 4},
		{0, 4}, {1, 5}, {2, 6}, {3, 7},
	} };
};

#endif