#ifndef GRID_H
#define GRID_H

#include "math/vec.h"
#include <array>
#include <vector>
#include <functional>
#include <Eigen/Sparse>
#include "isosurface.h"

struct Grid
{
	std::array<Vec3, 8> pos;
	std::array<Real, 8> valueOfPos;

	static const inline std::array<std::pair<int, int>, 12> edgeIdx = { {
		{0, 1}, {1, 2}, {2, 3}, {3, 0},
		{4, 5}, {5, 6}, {6, 7}, {7, 4},
		{0, 4}, {1, 5}, {2, 6}, {3, 7},
	} };
};

struct Cells
{
	Cells(Real begin, Real end, int resolution):
	m_begin(begin),
	m_end(end),
	m_resolution(resolution),
	m_gridSize((end - begin) / resolution),
	m_epsilon(1e-4f),
	m_pointVal((resolution + 1) * (resolution + 1) * (resolution + 1)) {}

	void marchingCubes(std::function<Real(const Vec3&)> SDF, std::vector<Vec3>& vertices, std::vector<std::vector<int>>& faces);
	void calculateGridPointVal(std::function<Real(const Vec3&)> SDF);
	void calculateIntersection(std::vector<Vec3>& vertices, Eigen::SparseVector<int>& edgeToVertexTable);
	void calculateTopology(Eigen::SparseVector<int>& edgeToVerticesTable, std::vector<std::vector<int>>& faces);

	void dualContouring(IsoSurface& surface, std::vector<Vec3>& vertices, std::vector<std::vector<int>>& faces);

	void calculateIntersection(std::vector<Vec2>& intersections, std::vector<Vec3>& normals, std::vector<std::vector<int>>& faces,
		std::function<Vec3(const Vec3&)> normalAt, Eigen::SparseVector<int>& edgeToVertexTable);
	void calculateVertices(std::vector<Vec2>& intersections, std::vector<Vec3>& normals, Eigen::SparseVector<int>& edgeToVertexTable, 
		std::vector<Vec3>& vertices, Eigen::SparseVector<int>& gridToVertexTable);
	void calculateTopologyDualContouring(Eigen::SparseVector<int>& gridToVertexTable, std::vector<std::vector<int>>& faces);

	Real m_begin;
	Real m_end;
	int m_resolution;
	Real m_gridSize;
	Real m_epsilon;
	std::vector<Real> m_pointVal;
};

#endif