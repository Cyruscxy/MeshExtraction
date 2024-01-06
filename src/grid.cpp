#include "grid.h"
#include <set>
#include "table.h"
#include <Eigen/Sparse>
#include <geometrycentral/numerical/linear_solvers.h>
#include <iostream>
#include <algorithm>

void printSparseMatrix(const Eigen::SparseMatrix<Real>& mat)
{
	for (int row = 0; row < mat.rows(); ++row)
	{
		for (int col = 0; col < mat.cols(); ++col)
		{
			std::cout << mat.coeff(row, col) << " ";
		}
		std::cout << std::endl;
	}
}

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

void Cells::calculateTopology(Eigen::SparseVector<int>& edgeToVertexTable, std::vector<std::vector<int>>& faces)
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
					int vid0 = edgeToVertexTable.coeff(gridEdges[state[eid]]);
					int vid1 = edgeToVertexTable.coeff(gridEdges[state[eid + 1]]);
					int vid2 = edgeToVertexTable.coeff(gridEdges[state[eid + 2]]);
					faces.push_back({ vid0, vid1, vid2 });
				}
			}
		}
	}
}

void Cells::dualContouring(IsoSurface& surface, std::vector<Vec3>& vertices, std::vector<std::vector<int>>& faces)
{
	Eigen::SparseVector<int> edgeToVertexTable;
	Eigen::SparseVector<int> gridToVertexTable;
	int nEdges = m_resolution * (m_resolution + 1) * (m_resolution + 1) * 3;
	int nGrids = m_resolution * m_resolution * m_resolution;
	edgeToVertexTable.resize(nEdges);
	gridToVertexTable.resize(nGrids);

	std::vector<Vec2> intersections;
	std::vector<Vec3> normals;

	this->calculateGridPointVal(surface);
	this->calculateIntersection(
		intersections,
		normals,
		faces,
		std::bind(&IsoSurface::normalAt, &surface, std::placeholders::_1),
		edgeToVertexTable
	);
	this->calculateVertices(intersections, normals, edgeToVertexTable, vertices, gridToVertexTable);
	this->calculateTopologyDualContouring(gridToVertexTable, faces);

}


void Cells::calculateIntersection(
	std::vector<Vec2>& intersections, 
	std::vector<Vec3>& normals, 
	std::vector<std::vector<int>>& faces, 
	std::function<Vec3(const Vec3&)> normalAt, 
	Eigen::SparseVector<int>& edgeToVertexTable
)
{
	int edgeTableOffset = m_resolution * (m_resolution + 1) * (m_resolution + 1);

	std::array<int, 3> pointIdxStride{ 1, m_resolution + 1, (m_resolution + 1) * (m_resolution + 1) };
	std::array<int, 3> edgeIdxStride{ 1, m_resolution, m_resolution * (m_resolution + 1) };
	std::array<int, 3> gridIdxStride{ 1, m_resolution, m_resolution * m_resolution };
 	std::array<int, 3> pointDimIdxStride;
	std::array<int, 3> gridDimIdxStride;

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

		gridDimIdxStride[0] = gridIdxStride[dim];
		gridDimIdxStride[1] = gridIdxStride[(dim + 1) % 3];
		gridDimIdxStride[2] = gridIdxStride[(dim + 2) % 3];

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
					auto inteVert = lerp(tail, head, tailVal, headVal);
					intersections.emplace_back(tailVal, headVal);
					normals.push_back(normalAt(inteVert));
					tail = head;
					tailVal = headVal;
					tailPointIdx = headPointIdx;

					// record topology
					if ( headVal > 0 )
					{
						/*faces.push_back({
							i * gridDimIdxStride[0] + (j - 1) * gridDimIdxStride[1] + (k - 1) * gridDimIdxStride[2],
							i * gridDimIdxStride[0] + (j - 1) * gridDimIdxStride[1] + k * gridDimIdxStride[2],
							i * gridDimIdxStride[0] + j * gridDimIdxStride[1] + k * gridDimIdxStride[2]
						});
						faces.push_back({
							i * gridDimIdxStride[0] + (j - 1) * gridDimIdxStride[1] + k * gridDimIdxStride[2],
							i * gridDimIdxStride[0] + j * gridDimIdxStride[1] + k * gridDimIdxStride[2],
							i * gridDimIdxStride[0] + j * gridDimIdxStride[1] + (k - 1) * gridDimIdxStride[2]
						});*/
						faces.push_back({
							i * gridDimIdxStride[0] + (j - 1) * gridDimIdxStride[1] + (k - 1) * gridDimIdxStride[2],
							i * gridDimIdxStride[0] + (j - 1) * gridDimIdxStride[1] + k * gridDimIdxStride[2],
							i * gridDimIdxStride[0] + j * gridDimIdxStride[1] + k * gridDimIdxStride[2],
							i * gridDimIdxStride[0] + j * gridDimIdxStride[1] + (k - 1) * gridDimIdxStride[2]
						});
					}
					else
					{
						/*faces.push_back({
							i * gridDimIdxStride[0] + j * gridDimIdxStride[1] + k * gridDimIdxStride[2],
							i * gridDimIdxStride[0] + (j - 1) * gridDimIdxStride[1] + k * gridDimIdxStride[2],
							i * gridDimIdxStride[0] + (j - 1) * gridDimIdxStride[1] + (k - 1) * gridDimIdxStride[2]
						});
						faces.push_back({
							i * gridDimIdxStride[0] + j * gridDimIdxStride[1] + (k - 1) * gridDimIdxStride[2],
							i * gridDimIdxStride[0] + j * gridDimIdxStride[1] + k * gridDimIdxStride[2],
							i * gridDimIdxStride[0] + (j - 1) * gridDimIdxStride[1] + k * gridDimIdxStride[2],
						});*/
						faces.push_back({
							i * gridDimIdxStride[0] + j * gridDimIdxStride[1] + (k - 1) * gridDimIdxStride[2],
							i * gridDimIdxStride[0] + j * gridDimIdxStride[1] + k * gridDimIdxStride[2],
							i * gridDimIdxStride[0] + (j - 1) * gridDimIdxStride[1] + k * gridDimIdxStride[2],
							i * gridDimIdxStride[0] + (j - 1) * gridDimIdxStride[1] + (k - 1) * gridDimIdxStride[2]
						});
					}
				}

				jIdx += pointDimIdxStride[1];
			}

			kIdx += pointDimIdxStride[2];
		}
	}
}

void Cells::calculateVertices(
	std::vector<Vec2>& intersections, 
	std::vector<Vec3>& normals, 
	Eigen::SparseVector<int>& edgeToVertexTable, 
	std::vector<Vec3>& vertices,
	Eigen::SparseVector<int>& gridToVertexTable
)
{

	const Vec3 cube[8] = {
		{-1.0f, -1.0f, -1.0f},
		{ 1.0f, -1.0f, -1.0f},
		{ 1.0f,  1.0f, -1.0f},
		{-1.0f,  1.0f, -1.0f},
		{-1.0f, -1.0f,  1.0f},
		{ 1.0f, -1.0f,  1.0f},
		{ 1.0f,  1.0f,  1.0f},
		{-1.0f,  1.0f,  1.0f}
	};

	constexpr std::pair<int, int> edgeIdx[12] = {
		{0, 1}, {1, 2}, {3, 2}, {0, 3},
		{4, 5}, {5, 6}, {7, 6}, {4, 7},
		{0, 4}, {1, 5}, {2, 6}, {3, 7}
	};

	int edgeTableOffset = m_resolution * (m_resolution + 1) * (m_resolution + 1);
	int edgeStridePerLayer = m_resolution * (m_resolution + 1);
	int edgeStridePerRow = m_resolution;
	int pointStridePerRow = m_resolution + 1;
	int pointStridePerLayer = pointStridePerRow * pointStridePerRow;
	std::array<int, 12> gridEdges;
	std::array<int, 8> gridPoints;
	std::vector<Eigen::Triplet<int>> gridToVertexCoeff;
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
				std::set<std::pair<int, int>> intersectionEdges;
				for (int eid = 0; state[eid] != -1; eid += 1)
				{
					intersectionEdges.insert({ state[eid], edgeToVertexTable.coeff(gridEdges[state[eid]]) });
				}
				
				Eigen::SparseMatrix<Real> A(intersectionEdges.size(), 3);
				Eigen::SparseMatrix<Real> b(intersectionEdges.size(), 1);
				int idx = 0;
				std::vector<Eigen::Triplet<Real>> coeff;
				std::vector<Eigen::Triplet<Real>> bCoeff;
				for ( auto [cubeEdgeIdx, globalEdgeIdx] : intersectionEdges )
				{
					Vec3& normal = normals[globalEdgeIdx];
					Vec2& weight = intersections[globalEdgeIdx];
					const Vec3& tail = cube[edgeIdx[cubeEdgeIdx].first];
					const Vec3& head = cube[edgeIdx[cubeEdgeIdx].second];
					Vec3 point = lerp(tail, head, weight[0], weight[1]);

					coeff.emplace_back(idx, 0, normal.x);
					coeff.emplace_back(idx, 1, normal.y);
					coeff.emplace_back(idx, 2, normal.z);
					Real nDotP = dot(normal, point);
					bCoeff.emplace_back(idx, 0, nDotP);
					idx += 1;
				}
				A.setFromTriplets(coeff.begin(), coeff.end());
				b.setFromTriplets(bCoeff.begin(), bCoeff.end());

				Eigen::SparseMatrix<Real> mat(3, 3);
				mat = A.transpose() * A;
				Eigen::SparseVector<Real> rhs = A.transpose() * b;
				Eigen::Matrix<Real, Eigen::Dynamic, 1> vert;

				geometrycentral::Solver<Real> solver(mat);
				//geometrycentral::SquareSolver<Real> solver(A);
				solver.solve(vert, rhs);
				// gridToVertexCoeff.emplace_back(i + j * m_resolution + k * m_resolution * m_resolution, 0, vertices.size()) ;
				int gridIdx = i + j * m_resolution + k * m_resolution * m_resolution;
				gridToVertexTable.insert(gridIdx) = vertices.size();
				/*Vec3 offset(
					 std::clamp(vert[0], -1.0f + m_epsilon, 1.0f - m_epsilon),
					 std::clamp(vert[1], -1.0f + m_epsilon, 1.0f - m_epsilon),
					 std::clamp(vert[2], -1.0f + m_epsilon, 1.0f - m_epsilon)
				);*/
				Vec3 offset(vert[0], vert[1], vert[2]);
				offset *= m_gridSize / 2;
				offset += (Vec3(i, j, k) + Vec3(0.5f)) * m_gridSize + Vec3(m_begin);
				vertices.push_back(offset);
			}
		}
	}
}

void Cells::calculateTopologyDualContouring(Eigen::SparseVector<int>& gridToVertexTable, std::vector<std::vector<int>>& faces)
{
	std::vector<char> facesMask;
	for ( auto& face : faces )
	{
		std::set<int> verticesCount;
		facesMask.push_back(0);
		for ( auto& idx : face )
		{
			idx = gridToVertexTable.coeff(idx, 0);
			if ( verticesCount.find(idx) != verticesCount.end() )
			{
				facesMask.back() = 1;
			}
			else
			{
				verticesCount.insert(idx);
			}
		}
	}

	// cull invalid faces
	int head = 0;
	int tail = facesMask.size() - 1;
	auto headMarching = [&facesMask, &head]()
	{
		for (; head < facesMask.size(); ++head)
		{
			if (facesMask[head] == 1) break;
		}
	};
	auto tailMarching = [&facesMask, &tail]()
	{
		for (; tail >= 0; --tail)
		{
			if (facesMask[tail] == 0) break;
		}
	};
	headMarching();
	tailMarching();

	while ( tail > head )
	{
		std::iter_swap(faces.begin() + head, faces.begin() + tail);
		facesMask[head] = 0;
		facesMask[tail] = 1;
		headMarching();
		tailMarching();
	}

	int nValidFaces = tail + 1;
	if (nValidFaces < faces.size()) faces.resize(nValidFaces);
}

