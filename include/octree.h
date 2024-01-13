#ifndef OCTREE_H
#define OCTREE_H

#include "math/vec.h"
#include <functional>
#include <array>
#include <memory>

enum OctreeNodeType
{
	INTERNAL_NODE = 0,
	LEAF_NODE = 1
};

struct OctreeNodeBase
{
public:
	OctreeNodeBase() = default;
	virtual ~OctreeNodeBase() = default;
	virtual OctreeNodeType getType() = 0;

private:
	// helper function
	static std::array<Vec3, 8> genPoints(const std::array<Vec3, 2>&  diagPoint);
	void calculateCubeIdx(const std::function<Real(const Vec3&)>& sdf);

private:
	std::array<Vec3, 2> m_diagPoint; // {vmin, vmax}
	// std::array<Real, 8> m_pointVal;
	char m_cubeIdx;
};

struct LeafNode: public OctreeNodeBase
{
public:
	LeafNode(const Vec3& vMin, const Vec3& vMax);
	LeafNode(Vec3&& vMin, Vec3&& vMax);

	OctreeNodeType getType() override { return LEAF_NODE; }

private:
};

struct InternalNode
{
	
};

class OctreeNodeCreator
{
public:
	std::unique_ptr<OctreeNodeBase> genNode(const std::array<Vec3, 2>& diagPoint);
private:
};

#endif