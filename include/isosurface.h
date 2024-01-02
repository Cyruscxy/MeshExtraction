#ifndef ISOSURFACE_H
#define ISOSURFACE_H

#include <functional>
#include "math/vec.h"

class IsoSurface
{
public:
	IsoSurface() = default;
	IsoSurface(std::function<Real(const Vec3& v)>& scalarFunction, std::function<Vec3(const Vec3& v)>& normalFunction, Real e) :
	m_scalarFunction(scalarFunction),
	m_normalFunction(normalFunction),
	m_epsilon(e) {}

	Real operator()(const Vec3& v) { return m_scalarFunction(v); }
	Vec3 normalAt(const Vec3& v) { return m_normalFunction(v); }

private:
	std::vector<Real> m_value;
	std::function<Real(const Vec3& v)> m_scalarFunction;
	std::function<Vec3(const Vec3& v)> m_normalFunction;
	Real m_epsilon;
};

#endif