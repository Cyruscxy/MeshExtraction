#ifndef OCTREE_H
#define OCTREE_H

#include "math/vec.h"
#include <functional>
#include <array>
#include <memory>

#include "isosurface.h"

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

	std::array<Vec3, 2> m_diagPoint; // {vmin, vmax}
	// std::array<Real, 8> m_pointVal;
	int m_cubeIdx;
};

struct LeafNode: public OctreeNodeBase
{
public:
	LeafNode(const Vec3& vMin, const Vec3& vMax);
	LeafNode(Vec3&& vMin, Vec3&& vMax);

	OctreeNodeType getType() override { return LEAF_NODE; }

};

struct InternalNode
{
	
};

class OctreeNodeCreator
{
public:
	OctreeNodeCreator(const IsoSurface& surface): m_surface(surface) {}
	std::unique_ptr<OctreeNodeBase> genNode(const std::array<Vec3, 2>& diagPoint);
private:
	static std::array<Vec3, 8> genPointsForGrid(const std::array<Vec3, 2>& diagPoint);
	int calculateCubeIdx(const std::array<Vec3, 8>& gridPoints, const std::function<Real(const Vec3&)>& sdf, std::array<Real, 8>& pointVal);

	IsoSurface m_surface;
};

#endif