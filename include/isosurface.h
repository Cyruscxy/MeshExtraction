#ifndef ISOSURFACE_H
#define ISOSURFACE_H

#include <functional>
#include "math/vec.h"

class IsoSurface
{
public:
	IsoSurface() = default;
	IsoSurface(std::function<Real(const Vec3& v)>& function, Real e) :
	m_scalarFunction(function),
	m_epsilon(e) {}

	void polygonizeGridMarchingCubes();

	std::vector<Vec3> m_vertices;
	std::vector<std::vector<int>> m_indices;

private:
	std::vector<Real> m_value;
	std::function<Real(const Vec3& v)> m_scalarFunction;
	Real m_epsilon;
};

#endif