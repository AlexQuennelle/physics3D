#include "collider.h"

#include <cstring>
#include <iostream>
#include <optional>
#include <raylib.h>
#include <raymath.h>
#include <vector>
#ifndef NDEBUG
#include "utils.h"
#include <ostream>
#endif // !NDEBUG

namespace phys
{

using std::vector;
#ifndef NDEBUG
using std::ostream;
#endif

std::optional<HitObj> CheckCollision(const Collider& col1, const Matrix trans1,
									 const Collider& col2, const Matrix trans2)
{
	vector<Collider> cols1 = col1.GetTransformed(trans1);
	vector<Collider> cols2 = col2.GetTransformed(trans2);
	return {};
}

MeshCollider::MeshCollider(const vector<Vector3>& verts,
						   const vector<Vector3>& nors)
{
	this->vertices.reserve(verts.size());
	std::memcpy(this->vertices.data(), &verts, verts.size());
	this->normals.reserve(nors.size());
	std::memcpy(this->normals.data(), &nors, nors.size());
}
//MeshCollider::MeshCollider(const MeshCollider& other)
//{
//	this->vertices = other.vertices;
//}

vector<Collider> MeshCollider::GetTransformed(const Matrix trans) const
{
	vector<Collider> cols{MeshCollider(*this) * trans};
	return cols;
}
MeshCollider MeshCollider::operator*(const Matrix mat)
{
	vector<Vector3> newVerts = this->vertices;
	for (auto vert : newVerts)
	{
		vert = Vector3Transform(vert, mat);
	}
	vector<Vector3> newNors = this->normals;
	for (auto nor : newNors)
	{
		nor = Vector3Transform(nor, mat);
	}
	return {newVerts, newNors};
}
vector<Collider> CompoundCollider::GetTransformed(const Matrix trans) const
{
	vector<Collider> cols;
	for (const auto& elem : this->colliders)
	{
		vector<Collider> transformed = elem.GetTransformed(trans);
		cols.insert(cols.end(), transformed.begin(), transformed.end());
	}
	return cols;
}

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

#ifndef NDEBUG
ostream& operator<<(ostream& ostr, HitObj hit)
{
	ostr << hit.HitPos << '\n';
	return ostr;
}
ostream& operator<<(ostream& ostr, Vector3 vec)
{
	ostr << '(' << vec.x << ", " << vec.y << ", " << vec.z << ')';
	return ostr;
}
#endif // !NDEBUG

} //namespace phys
