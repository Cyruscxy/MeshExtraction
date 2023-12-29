#ifndef MARCHING_CUBES_H
#define MARCHING_CUBES_H

#include <vector>
#include "grid.h"

void marchingCubesPolygonize(Grid& grid, std::vector<Vec3>& vertices);

#endif