#ifndef OCTREE_H
#define OCTREE_H

#include "math/vec.h"
#include <functional>
#include <array>

enum OctreeNodeType
{
	INTERNAL_NODE = 0,
	LEAF_NODE = 1
};

class OctreeNodeBase
{
public:
	OctreeNodeBase() = default;
	virtual ~OctreeNodeBase() = default;
	virtual OctreeNodeType getType() = 0;

private:
	// helper function
	std::array<Vec3, 8> genPoints();
	void calculateCubeIdx(const std::function<Real(const Vec3&)>& sdf);

private:
	std::array<Vec3, 2> m_diagPoint; // {vmin, vmax}
	std::array<Real, 8> m_pointVal;
	char m_cubeIdx;
};

class LeafNode: public OctreeNodeBase
{
public:
	LeafNode(const Vec3& vMin, const Vec3& vMax);
	LeafNode(Vec3&& vMin, Vec3&& vMax);

	OctreeNodeType getType() override { return LEAF_NODE; }

private:
};

class Octree
{
	
};

#endif