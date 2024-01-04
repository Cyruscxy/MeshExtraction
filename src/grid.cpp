#include "grid.h"
#include "table.h"
#include <Eigen/Sparse>

void Cells::marchingCubes(std::function<Real(const Vec3&)> SDF, std::vector<Vec3>& vertices, std::vector<std::vector<int>>& faces)
{
	Eigen::SparseVector<int> edgeToVertexTable;
	int nEdges = m_resolution * (m_resolution + 1) * (m_resolution + 1) * 3;
	edgeToVertexTable.resize(nEdges);

	this->calculateGridPointVal(SDF);
	this->calculateIntersection(vertices, edgeToVertexTable);
	this->calculateTopology(edgeToVertexTable, faces);
}

void Cells::calculateGridPointVal(std::function<Real(const Vec3&)> SDF)
{
	int nGridPoint = (m_resolution + 1) * (m_resolution + 1) * (m_resolution + 1);
	m_pointVal.resize(nGridPoint);
	Vec3 point(m_begin, m_begin, m_begin);
	int idx = 0;
	for (int z = 0; z < m_resolution + 1; ++z)
	{
		for (int y = 0; y < m_resolution + 1; ++y)
		{
			for (int x = 0; x < m_resolution + 1; ++x)
			{
				m_pointVal[idx] = SDF(point);
				idx += 1;
				point.x += m_gridSize;
			}
			point.x = m_begin;
			point.y += m_gridSize;
		}
		point.y = m_begin;
		point.z += m_gridSize;
	}
}

void Cells::calculateIntersection(std::vector<Vec3>& vertices, Eigen::SparseVector<int>& edgeToVertexTable)
{
	int edgeTableOffset = m_resolution * (m_resolution + 1) * (m_resolution + 1);

	std::array<int, 3> pointIdxStride{ 1, m_resolution + 1, (m_resolution + 1) * (m_resolution + 1) };
	std::array<int, 3> edgeIdxStride{ 1, m_resolution, m_resolution * (m_resolution + 1) };
	std::array<int, 3> pointDimIdxStride;

	for (int dim = 0; dim < 3; ++dim) // x, y, z dim
	{
		Vec3 dir;
		dir[dim] = m_gridSize;

		// when dim is x, j -> y, k -> z
		// when dim is y, j -> z, k -> x
		// when dim is z, j -> x, k -> y
		pointDimIdxStride[0] = pointIdxStride[dim];
		pointDimIdxStride[1] = pointIdxStride[(dim + 1) % 3];
		pointDimIdxStride[2] = pointIdxStride[(dim + 2) % 3];

		Vec3 iOffset, jOffset, kOffset;
		iOffset[dim] = m_gridSize;
		jOffset[(dim + 1) % 3] = m_gridSize;
		kOffset[(dim + 2) % 3] = m_gridSize;

		int kIdx = 0;
		for (int k = 0; k < m_resolution + 1; ++k)
		{
			int jIdx = 0;
			for (int j = 0; j < m_resolution + 1; ++j)
			{
				int tailPointIdx = jIdx + kIdx;
				Real tailVal = m_pointVal[tailPointIdx];
				Vec3 tail = Vec3(m_begin) + jOffset * j + kOffset * k;

				for (int i = 0; i < m_resolution; ++i)
				{
					int headPointIdx = tailPointIdx + pointDimIdxStride[0];
					Real headVal = m_pointVal[headPointIdx];
					Vec3 head = tail + iOffset;

					if (headVal * tailVal > 0) {
						tail = head;
						tailVal = headVal;
						tailPointIdx = headPointIdx;
						continue;
					}

					int edgeIdx = i * edgeIdxStride[0] + j * edgeIdxStride[1] + k * edgeIdxStride[2];
					edgeIdx += dim * edgeTableOffset;
					edgeToVertexTable.insert(edgeIdx) = vertices.size();
					vertices.push_back(lerp(tail, head, tailVal, headVal));
					tail = head;
					tailVal = headVal;
					tailPointIdx = headPointIdx;
				}

				jIdx += pointDimIdxStride[1];
			}

			kIdx += pointDimIdxStride[2];
		}
	}
}

void Cells::calculateTopology(Eigen::SparseVector<int>& edgeToVerticesTable, std::vector<std::vector<int>>& faces)
{
	int edgeTableOffset = m_resolution * (m_resolution + 1) * (m_resolution + 1);
	int edgeStridePerLayer = m_resolution * (m_resolution + 1);
	int edgeStridePerRow = m_resolution;
	int pointStridePerRow = m_resolution + 1;
	int pointStridePerLayer = pointStridePerRow * pointStridePerRow;
	std::array<int, 12> gridEdges;
	std::array<int, 8> gridPoints;
	for (int k = 0; k < m_resolution; ++k)
	{
		for (int j = 0; j < m_resolution; ++j)
		{
			for (int i = 0; i < m_resolution; ++i)
			{
				// get idx of points corresponding to current grid
				gridPoints[0] = i + j * pointStridePerRow + k * pointStridePerLayer;
				gridPoints[1] = i + 1 + j * pointStridePerRow + k * pointStridePerLayer;
				gridPoints[2] = i + 1 + (j + 1) * pointStridePerRow + k * pointStridePerLayer;
				gridPoints[3] = i + (j + 1) * pointStridePerRow + k * pointStridePerLayer;
				gridPoints[4] = i + j * pointStridePerRow + (k + 1) * pointStridePerLayer;
				gridPoints[5] = i + 1 + j * pointStridePerRow + (k + 1) * pointStridePerLayer;
				gridPoints[6] = i + 1 + (j + 1) * pointStridePerRow + (k + 1) * pointStridePerLayer;
				gridPoints[7] = i + (j + 1) * pointStridePerRow + (k + 1) * pointStridePerLayer;

				// get idx of edges corresponding to current grid
				int xStartEdge = i + j * edgeStridePerRow + k * edgeStridePerLayer;
				int yStartEdge = j + k * edgeStridePerRow + i * edgeStridePerLayer;
				int zStartEdge = k + i * edgeStridePerRow + j * edgeStridePerLayer;

				gridEdges[0] = xStartEdge;
				gridEdges[1] = yStartEdge + edgeStridePerLayer + edgeTableOffset;
				gridEdges[2] = xStartEdge + edgeStridePerRow;
				gridEdges[3] = yStartEdge + edgeTableOffset;

				gridEdges[4] = xStartEdge + edgeStridePerLayer;
				gridEdges[5] = yStartEdge + edgeStridePerLayer + edgeStridePerRow + edgeTableOffset;
				gridEdges[6] = xStartEdge + edgeStridePerLayer + edgeStridePerRow;
				gridEdges[7] = yStartEdge + edgeStridePerRow + edgeTableOffset;

				gridEdges[8] = zStartEdge + edgeTableOffset * 2;
				gridEdges[9] = zStartEdge + edgeStridePerRow + edgeTableOffset * 2;
				gridEdges[10] = zStartEdge + edgeStridePerRow + edgeStridePerLayer + edgeTableOffset * 2;
				gridEdges[11] = zStartEdge + edgeStridePerLayer + edgeTableOffset * 2;

				// compute cube index to find grid state
				int cubeIdx = 0;
				for (int gridPointIdx = 0; gridPointIdx < 8; ++gridPointIdx)
				{
					if (m_pointVal[gridPoints[gridPointIdx]] < 0.0f) cubeIdx |= 1 << gridPointIdx;
				}

				if (edgeTable[cubeIdx] == 0) continue;

				char* state = triTable[cubeIdx];
				for (int eid = 0; state[eid] != -1; eid += 3)
				{
					int vid0 = edgeToVerticesTable.coeff(gridEdges[state[eid]]);
					int vid1 = edgeToVerticesTable.coeff(gridEdges[state[eid + 1]]);
					int vid2 = edgeToVerticesTable.coeff(gridEdges[state[eid + 2]]);
					faces.push_back({ vid0, vid1, vid2 });
				}
			}
		}
	}
}

void Cells::calculateIntersection(std::vector<Vec3>& intersections, std::vector<Vec3>& normals, std::function<Vec3(const Vec3&)>& normalAt, Eigen::SparseVector<int>& edgeToVertexTable)
{
	int edgeTableOffset = m_resolution * (m_resolution + 1) * (m_resolution + 1);

	std::array<int, 3> pointIdxStride{ 1, m_resolution + 1, (m_resolution + 1) * (m_resolution + 1) };
	std::array<int, 3> edgeIdxStride{ 1, m_resolution, m_resolution * (m_resolution + 1) };
	std::array<int, 3> pointDimIdxStride;

	for (int dim = 0; dim < 3; ++dim) // x, y, z dim
	{
		Vec3 dir;
		dir[dim] = m_gridSize;

		// when dim is x, j -> y, k -> z
		// when dim is y, j -> z, k -> x
		// when dim is z, j -> x, k -> y
		pointDimIdxStride[0] = pointIdxStride[dim];
		pointDimIdxStride[1] = pointIdxStride[(dim + 1) % 3];
		pointDimIdxStride[2] = pointIdxStride[(dim + 2) % 3];

		Vec3 iOffset, jOffset, kOffset;
		iOffset[dim] = m_gridSize;
		jOffset[(dim + 1) % 3] = m_gridSize;
		kOffset[(dim + 2) % 3] = m_gridSize;

		int kIdx = 0;
		for (int k = 0; k < m_resolution + 1; ++k)
		{
			int jIdx = 0;
			for (int j = 0; j < m_resolution + 1; ++j)
			{
				int tailPointIdx = jIdx + kIdx;
				Real tailVal = m_pointVal[tailPointIdx];
				Vec3 tail = Vec3(m_begin) + jOffset * j + kOffset * k;

				for (int i = 0; i < m_resolution; ++i)
				{
					int headPointIdx = tailPointIdx + pointDimIdxStride[0];
					Real headVal = m_pointVal[headPointIdx];
					Vec3 head = tail + iOffset;

					if (headVal * tailVal > 0) {
						tail = head;
						tailVal = headVal;
						tailPointIdx = headPointIdx;
						continue;
					}

					int edgeIdx = i * edgeIdxStride[0] + j * edgeIdxStride[1] + k * edgeIdxStride[2];
					edgeIdx += dim * edgeTableOffset;
					edgeToVertexTable.insert(edgeIdx) = intersections.size();
					intersections.push_back(lerp(tail, head, tailVal, headVal));
					normals.push_back(normalAt(intersections.back()));
					tail = head;
					tailVal = headVal;
					tailPointIdx = headPointIdx;
				}

				jIdx += pointDimIdxStride[1];
			}

			kIdx += pointDimIdxStride[2];
		}
	}
}

void Cells::calculateVertices(std::vector<Vec3>& intersections, std::vector<Vec3>& normals, Eigen::SparseVector<int>& edgeToVertexTable, std::vector<Vec3>& vertices)
{
	
}

