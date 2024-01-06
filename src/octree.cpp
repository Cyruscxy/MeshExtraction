#include "octree.h"



std::array<Vec3, 8> OctreeNodeBase::genPoints()
{
	std::array<Vec3, 8> points;
}


LeafNode::LeafNode(const Vec3& vMin, const Vec3& vMax): m_min(vMin), m_max(vMax), m_cubeIdx(0) { }
LeafNode::LeafNode(Vec3&& vMin, Vec3&& vMax) : LeafNode(vMin, vMax) { }
