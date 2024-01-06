#ifndef OCTREE_H
#define OCTREE_H

#include "math/vec.h"
#include <functional>
#include <array>

enum OctreeNodeType
{
	LEAF_NODE,
	INTERN_NODE
};

class OctreeNodeBase
{
public:
	OctreeNodeBase() = default;
	virtual ~OctreeNodeBase() = default;
	virtual OctreeNodeType getType() = 0;

	void calculateCubeIdx(const std::function<Real(const Vec3&)>& sdf);

private:
	// helper function
	std::array<Vec3, 8> genPoints();

private:
	Vec3 m_min;
	Vec3 m_max;
	std::array<Real, 8> m_pointVal;
	int m_cubeIdx;
};

class LeafNode: public OctreeNodeBase
{
public:
	LeafNode(const Vec3& vMin, const Vec3& vMax);
	LeafNode(Vec3&& vMin, Vec3&& vMax);

	OctreeNodeType getType() override { return LEAF_NODE; }

private:
	Vec3 m_min;
	Vec3 m_max;
	int m_cubeIdx;
};

class Octree
{
	
};

#endif