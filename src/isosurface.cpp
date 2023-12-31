#include "isosurface.h"
#include "grid.h"
#include "table.h"

namespace MarchingCubesConfig
{
	constexpr int resolution = 2;
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

void IsoSurface::polygonizeGrid()
{
	using namespace MarchingCubesConfig;
	int nEdges = resolution * (resolution + 1) * (resolution + 1) * 3;
	int nGridPoint = (resolution + 1) * (resolution + 1) * (resolution + 1);
	std::vector<int> edgeToVerticesTable(nEdges, -1);
	std::vector<Real> gridPointVal(nGridPoint);

	// calculate the scalar value of all grid point
	Vec3 point(begin, begin, begin);
	int idx = 0;
	for ( int z = 0; z < resolution + 1; ++z )
	{
		for ( int y = 0; y < resolution + 1; ++y )
		{
			for ( int x = 0; x < resolution + 1; ++x )
			{
				gridPointVal[idx] = m_scalarFunction(point);
				idx += 1;
				point.x += gridSize;
			}
			point.x = begin;
			point.y += gridSize;
		}
		point.y = begin;
		point.z += gridSize;
	}

	// calculate vertices resident on every edges
	std::array<int, 3> idxOffset {1, resolution + 1, (resolution + 1) * (resolution + 1)};
	std::array<int, 3> dimIdxStride;
	for ( int dim = 0; dim < 3; ++dim ) // x, y, z dim
	{
		Vec3 dir;
		dir[dim] = gridSize;

		// when dim is x, j -> y, k -> z
		// when dim is y, j -> z, k -> x
		// when dim is z, j -> x, k -> y
		dimIdxStride[0] = idxOffset[dim];
		dimIdxStride[1] = idxOffset[(dim + 1) % 3];
		dimIdxStride[2] = idxOffset[(dim + 2) % 3];

		Vec3 iOffset, jOffset, kOffset;
		iOffset[dim] = gridSize;
		jOffset[(dim + 1) % 3] = gridSize;
		kOffset[(dim + 2) % 3] = gridSize;

		int kIdx = 0;
		for ( int k = 0; k < resolution + 1; ++k )
		{
			int jIdx = 0;
			for ( int j = 0; j < resolution + 1; ++j )
			{
				int tailPointIdx = jIdx + kIdx;
				Real tailVal = gridPointVal[tailPointIdx];
				Vec3 tail = Vec3(begin) + jOffset * j + kOffset * k;

				for ( int i = 0; i < resolution; ++i )
				{
					int headPointIdx = tailPointIdx + dimIdxStride[0];
					Real headVal = gridPointVal[headPointIdx];
					Vec3 head = tail + iOffset;

					if (headVal * tailVal > 0) {
						tail = head;
						tailVal = headVal;
						tailPointIdx = headPointIdx;
						continue;
					}

					edgeToVerticesTable[tailPointIdx] = m_vertices.size();
					m_vertices.push_back(lerp(tail, head, tailVal, headVal));
					tail = head;
					tailVal = headVal;
					tailPointIdx = headPointIdx;
				}

				jIdx += dimIdxStride[1];
			}

			kIdx += dimIdxStride[2];
		}
	}

	// get the topology of the mesh, connect the vertices
	int edgeStridePerLayer = resolution * (resolution + 1);
	int edgeStridePerRow = resolution;
	int pointStridePerRow = resolution + 1;
	int pointStridePerLayer = pointStridePerRow * pointStridePerRow;
	std::array<int, 12> gridEdges;
	std::array<int, 8> gridPoints;
	for ( int k = 0; k < resolution; ++k )
	{
		for ( int j = 0; j < resolution; ++j )
		{
			for ( int i = 0; i < resolution; ++i )
			{
				// get idx of points corresponding to current grid
				gridPoints[0] = i + j * pointStridePerRow + k * pointStridePerLayer;
				gridPoints[1] = i + 1 + j * pointStridePerRow + k * pointStridePerLayer;
				gridPoints[2] = i + (j + 1) * pointStridePerRow + k * pointStridePerLayer;
				gridPoints[3] = i + 1 + (j + 1) * pointStridePerRow + k * pointStridePerLayer;
				gridPoints[4] = i + j * pointStridePerRow + (k + 1) * pointStridePerLayer;
				gridPoints[5] = i + 1 + j * pointStridePerRow + (k + 1) * pointStridePerLayer;
				gridPoints[6] = i + (j + 1) * pointStridePerRow + (k + 1) * pointStridePerLayer;
				gridPoints[7] = i + 1 + (j + 1) * pointStridePerRow + (k + 1) * pointStridePerLayer;

				// get idx of edges corresponding to current grid
				int xStartEdge = i + j * edgeStridePerRow + k * edgeStridePerLayer;
				int yStartEdge = j + k * edgeStridePerRow + i * edgeStridePerLayer;
				int zStartEdge = k + i * edgeStridePerRow + j * edgeStridePerLayer;

				gridEdges[0] = xStartEdge;
				gridEdges[1] = yStartEdge + edgeStridePerLayer;
				gridEdges[2] = xStartEdge + edgeStridePerRow;
				gridEdges[3] = yStartEdge;

				gridEdges[4] = xStartEdge + edgeStridePerLayer;
				gridEdges[5] = yStartEdge + edgeStridePerLayer + edgeStridePerRow;
				gridEdges[6] = xStartEdge + edgeStridePerLayer + edgeStridePerRow;
				gridEdges[7] = yStartEdge + edgeStridePerRow;

				gridEdges[8] = zStartEdge;
				gridEdges[9] = zStartEdge + edgeStridePerRow;
				gridEdges[10] = zStartEdge + edgeStridePerRow + edgeStridePerLayer;
				gridEdges[11] = zStartEdge + edgeStridePerLayer;

				// compute cube index to find grid state
				int cubeIdx = 0;
				for ( int gridPointIdx = 0; gridPointIdx < 8; ++gridPointIdx)
				{
					if (gridPointVal[gridPoints[gridPointIdx]] < 0.0f) cubeIdx |= 1 << gridPointIdx;
				}

				if (edgeTable[cubeIdx] == 0) continue;

				char* state = triTable[cubeIdx];
				for ( int eid = 0; state[eid] != -1; eid += 3 )
				{
					int vid0 = edgeToVerticesTable[gridEdges[state[eid]]];
					int vid1 = edgeToVerticesTable[gridEdges[state[eid + 1]]];
					int vid2 = edgeToVerticesTable[gridEdges[state[eid + 2]]];
					m_indices.push_back({vid0, vid1, vid2});
				}
			}
		}
	}

}

