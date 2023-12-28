#ifndef GRID_H
#define GRID_H

#include "math/vec.h"
#include <array>

struct Grid
{
	Vec3 minPos;
	Vec3 maxPos;
	std::array<Real, 8> valueOfPos;
};

#endif