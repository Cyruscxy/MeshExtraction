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

	IsoSurface surface(sphere, 1e-4f);
	surface.polygonizeGrid();

	polyscope::init();

	auto handle = polyscope::registerSurfaceMesh("Sphere", surface.m_vertices, surface.m_indices);

	polyscope::show();

	return 0;
}