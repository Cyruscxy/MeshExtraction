#include <iostream>
#include <polyscope/polyscope.h>
#include <polyscope/curve_network.h>
#include <polyscope/surface_mesh.h>
#include "isosurface.h"

int main()
{
	std::function<Real(const Vec3& v)> sphere;
	sphere = [](const Vec3& v) -> Real
	{
		return v.norm2() - 0.5f;
	};

	std::function<Real(const Vec3& v)> crazyFunction;
	crazyFunction = [](const Vec3& v) -> Real
	{
		auto P = v * 3.0f;
		return sinf(P.x * P.y + P.x * P.z + P.y * P.z) + sinf(P.x * P.y) + sinf(P.y * P.z) + sinf(P.x * P.z) - 1.0f;
	};

	IsoSurface surface(crazyFunction, 1e-4f);
	surface.polygonizeGridMarchingCubes();

	polyscope::init();

	auto handle = polyscope::registerSurfaceMesh("Sphere", surface.m_vertices, surface.m_indices);

	polyscope::show();

	return 0;
}