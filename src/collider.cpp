#include "collider.h"

#include <raylib.h>
#include <raymath.h>
#include <vector>
#ifndef NDEBUG
#include <ostream>
#endif // !NDEBUG

namespace phys
{

using std::vector;
#ifndef NDEBUG
using std::ostream;
#endif

MeshCollider CreateBoxCollider(Matrix transform)
{
	vector<Vector3> verts{
		{.x = 0.0f, .y = 0.0f, .z = 0.0f}, {.x = 1.0f, .y = 0.0f, .z = 0.0f},
		{.x = 0.0f, .y = 1.0f, .z = 0.0f}, {.x = 1.0f, .y = 0.0f, .z = 1.0f},
		{.x = 0.0f, .y = 0.0f, .z = 1.0f}, {.x = 1.0f, .y = 1.0f, .z = 0.0f},
		{.x = 0.0f, .y = 1.0f, .z = 1.0f}, {.x = 1.0f, .y = 1.0f, .z = 1.0f}};
	vector<Vector3> nors{{.x = 1.0f, .y = 0.0f, .z = 0.0f},
						 {.x = 0.0f, .y = 1.0f, .z = 0.0f},
						 {.x = 0.0f, .y = 0.0f, .z = 1.0f}};
	for (auto vert : verts)
	{
		Vector3Transform(vert, transform);
	}
	for (auto nor : nors)
	{
		Vector3Transform(nor, transform);
	}
	return *new MeshCollider(verts, nors);
}

} //namespace phys
