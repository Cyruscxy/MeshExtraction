#include "marching_cubes.h"
#include "table.h"

void marchingCubesPolygonize(Grid& grid, std::vector<Vec3>& vertices, std::vector<int>& indices)
{
	
	int currentVerticesCount = vertices.size();
	int cubeIdx = 0;

	std::array<Vec3, 12> verticesList;
	std::vector<int8_t> localRemap(12, -1);
	std::vector<int> newVerticesIdx;

	for ( int i = 0; i < 8; ++i )
	{
		if (grid.valueOfPos[i] < 0.0f) cubeIdx |= 1 << i;
	}

	if (edgeTable[cubeIdx] == 0) return;

	for ( int i = 0; i < 12; ++i )
	{
		int mask = 1 << (i + 1);
		auto& [start, end] = Grid::edgeIdx[i];
		if ( edgeTable[cubeIdx] & mask )
		{
			verticesList[i] = lerp(grid.pos[start], grid.pos[end], grid.valueOfPos[start], grid.valueOfPos[end]);
		}
	}

	char newVerticesCount = 0;
	for ( int i = 0; triTable[cubeIdx][i] != -1; ++i )
	{
		if ( localRemap[triTable[cubeIdx][i]] == -1 )
		{
			vertices.push_back(verticesList[triTable[cubeIdx][i]]);
			localRemap[triTable[cubeIdx][i]] = newVerticesCount;
			newVerticesCount += 1;
		}
	}

	for ( int i = 0; triTable[cubeIdx][i] != -1; i += 3 )
	{
		char xid = triTable[cubeIdx][i + 0];
		char yid = triTable[cubeIdx][i + 1];
		char zid = triTable[cubeIdx][i + 2];

		indices.push_back(localRemap[xid] + currentVerticesCount);
		indices.push_back(localRemap[yid] + currentVerticesCount);
		indices.push_back(localRemap[zid] + currentVerticesCount);
	}
}
