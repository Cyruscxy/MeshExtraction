#include <iostream>
#include <polyscope/polyscope.h>
#include <polyscope/curve_network.h>
#include <polyscope/surface_mesh.h>
#include "isosurface.h"
#include "grid.h"

int main()
{
	std::function<Real(const Vec3& v)> sphere;
	sphere = [](const Vec3& v) -> Real
	{
		return v.norm2() - 0.5f;
	};
	std::function<Vec3(const Vec3& v)> sphereNormal;
	sphereNormal = [](const Vec3& v) -> Vec3
	{
		return normalize(v);
	};

	std::function<Real(const Vec3& v)> crazyFunction;
	crazyFunction = [](const Vec3& v) -> Real
	{
		auto P = v * 3.0f;
		return sinf(P.x * P.y + P.x * P.z + P.y * P.z) + sinf(P.x * P.y) + sinf(P.y * P.z) + sinf(P.x * P.z) - 1.0f;
	};

	std::function<Real(const Vec3& v)> cube;
	cube = [](const Vec3& v) -> Real
	{
		Vec3 p = v.habs() - Vec3(0.5f);
		return std::max(p.x, std::max(p.y, p.z));
	};
	std::function<Vec3(const Vec3& v)> cubeNormal;
	cubeNormal = [](const Vec3& v) -> Vec3
	{
		Vec3 p = v.habs();
		Real distance = std::max(std::max(p.x, p.y), p.z);
		if (distance == p.x) return Vec3(1.0f, 0.0f, 0.0f);
		if (distance == p.y) return Vec3(0.0f, 1.0f, 0.0f);
		return Vec3(0.0f, 0.0f, 1.0f);
	};

	// IsoSurface surface(sphere, sphereNormal, 1e-4f);
	IsoSurface surface(cube, cubeNormal, 1e-4f);
	Cells cells(-1.0f, 1.0f, 4);
	std::vector<Vec3> vertices;
	std::vector<std::vector<int>> indices;
	// cells.marchingCubes(surface, vertices, indices);
	cells.dualContouring(surface, vertices, indices);
	// surface.polygonizeGridMarchingCubes();

	std::vector<std::array<int, 2>> edges;
	for ( auto& face : indices )
	{
		for ( int i = 0; i < face.size(); ++i )
		{
			edges.push_back({ face[i], face[(i + 1) % face.size()] });
		}
	}

	polyscope::init();

	// auto handle = polyscope::registerSurfaceMesh("Sphere", vertices, indices);

	auto handle = polyscope::registerCurveNetwork("Sphere", vertices, edges);

	polyscope::show();

	return 0;
}