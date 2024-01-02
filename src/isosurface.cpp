#include "isosurface.h"
#include "grid.h"
#include "table.h"

namespace MarchingCubesConfig
{
	constexpr int resolution = 64;
	constexpr Real begin = -1.0f;
	constexpr Real end = 1.0f;
	constexpr Real gridSize = (end - begin) / resolution;
	constexpr Real isoLevel = 0.5f;
}

int MortonEncodeSpread(int num)
{
	num = (num | (num << 16)) & 0x030000FF;
	num = (num | (num << 8)) & 0x0300F00F;
	num = (num | (num << 4)) & 0x030C30C3;
	num = (num | (num << 2)) & 0x09249249;
	return num;
}

int MortonEncode(int x, int y, int z)
{
	x = MortonEncodeSpread(x);
	y = MortonEncodeSpread(y);
	z = MortonEncodeSpread(z);
	return x | (y << 1) | (z << 2);
}
