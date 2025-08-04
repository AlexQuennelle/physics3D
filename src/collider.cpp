#include "collider.h"

#include <iostream>
#include <limits>
#include <optional>
#include <raylib.h>
#include <raymath.h>
#include <vector>
#ifndef NDEBUG
//#define VERBOSELOG_COL
#include "utils.h"
#include <ostream>
#endif // !NDEBUG

namespace phys
{

using std::vector;
#ifndef NDEBUG
using std::ostream;
#endif

std::optional<HitObj> CheckCollision(const Collider* col1, const Matrix trans1,
									 const Collider* col2, const Matrix trans2)
{
	vector<Collider*> cols1 = col1->GetTransformed(trans1);
	vector<Collider*> cols2 = col2->GetTransformed(trans2);
	bool collision = false;
	for (const auto& collider1 : cols1)
	{
		for (const auto& collider2 : cols2)
		{
			vector<Vector3> nors = collider1->GetNormals();
			vector<Vector3> nors2 = collider2->GetNormals();
			nors.insert(nors.end(), nors2.begin(), nors2.end());
			bool hit = true;
			for (const auto nor : nors)
			{
				Range proj1 = collider1->GetProjection(nor);
				Range proj2 = collider2->GetProjection(nor);
#ifdef VERBOSELOG_COL
				std::cout << proj1 << '\n';
				std::cout << proj2 << '\n';
#endif // !VERBOSELOG_COL
				bool overlapped =
					proj1.min <= proj2.max && proj2.min <= proj1.max;
				if (overlapped)
				{
#ifdef VERBOSELOG_COL
					SetTextColor({0, 255, 0, 255});
					std::cout << nor << " Hit!\n";
					ClearStyles();
#endif // !VERBOSELOG_COL
				}
				else
				{
#ifdef VERBOSELOG_COL
					SetTextColor({255, 255, 0, 255});
					std::cout << "Miss!\n";
					ClearStyles();
#endif // !VERBOSELOG_COL
					hit = false;
					break;
				}
			}
			collision |= hit;
		}
	}
	if (collision)
	{
		HitObj hitObj{
			.ThisCol = col1, .OtherCol = col2, .HitPos = {0.0f, 0.0f, 0.0f}};
		return hitObj;
	}
	return {};
}

MeshCollider::MeshCollider(const vector<Vector3>& verts,
						   const vector<Vector3>& nors)
{
	this->vertices.insert(this->vertices.end(), verts.begin(), verts.end());
	this->normals.insert(this->normals.end(), nors.begin(), nors.end());
}
// TODO: Remove unused function.
MeshCollider MeshCollider::operator*(const Matrix& mat)
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

vector<Collider*> MeshCollider::GetTransformed(const Matrix trans) const
{
#ifdef VERBOSELOG_COL
	std::cout << trans << '\n';
#endif // VERBOSELOG_COL
	vector<Vector3> newVerts;
	newVerts.reserve(this->vertices.size());
	for (auto vert : this->vertices)
	{
#ifdef VERBOSELOG_COL
		std::cout << vert << Vector3Transform(vert, trans);
		std::cout << '\n';
#endif
		newVerts.push_back(Vector3Transform(vert, trans));
	}
#ifdef VERBOSELOG_COL
	std::cout << '\n';
#endif
	vector<Vector3> newNors;
	newNors.reserve(this->normals.size());
	for (auto nor : this->normals)
	{
		newNors.push_back(Vector3Normalize(Vector3Transform(nor, trans)));
	}
	vector<Collider*> cols{new MeshCollider(newVerts, newNors)};
	return cols;
}
vector<Vector3> MeshCollider::GetNormals() const { return {this->normals}; }
Range MeshCollider::GetProjection(const Vector3 nor) const
{
	Range proj{
		.min = std::numeric_limits<float>::max(),
		.max = std::numeric_limits<float>::min(),
	};
	for (const auto vert : this->vertices)
	{
		float projected = Vector3DotProduct(vert, nor);
		proj.min = projected < proj.min ? projected : proj.min;
		proj.max = projected > proj.max ? projected : proj.max;
	}
	return proj;
}

vector<Collider*> CompoundCollider::GetTransformed(const Matrix trans) const
{
	vector<Collider*> cols;
	for (const auto& elem : this->colliders)
	{
		vector<Collider*> transformed = elem->GetTransformed(trans);
		cols.insert(cols.end(), transformed.begin(), transformed.end());
	}
	return cols;
}
vector<Vector3> CompoundCollider::GetNormals() const
{
	vector<Vector3> nors;
	for (const auto& col : this->colliders)
	{
		auto colNors = col->GetNormals();
		nors.insert(nors.end(), colNors.begin(), colNors.end());
	}
	return nors;
}

MeshCollider* CreateBoxCollider(Matrix transform)
{
#ifdef VERBOSELOG_COL
	std::cout << transform << '\n';
#endif
	static const vector<Vector3> verts{
		{.x = -0.5f, .y = -0.5f, .z = -0.5f},
		{.x = 0.5f, .y = -0.5f, .z = -0.5f},
		{.x = -0.5f, .y = 0.5f, .z = -0.5f},
		{.x = 0.5f, .y = -0.5f, .z = 0.5f},
		{.x = -0.5f, .y = -0.5f, .z = 0.5f},
		{.x = 0.5f, .y = 0.5f, .z = -0.5f},
		{.x = -0.5f, .y = 0.5f, .z = 0.5f},
		{.x = 0.5f, .y = 0.5f, .z = 0.5f},
	};
	vector<Vector3> newVerts;
	static const vector<Vector3> nors{
		{.x = 1.0f, .y = 0.0f, .z = 0.0f},
		{.x = 0.0f, .y = 1.0f, .z = 0.0f},
		{.x = 0.0f, .y = 0.0f, .z = 1.0f},
	};
	vector<Vector3> newNors;

	newVerts.reserve(verts.size());
	for (auto vert : verts)
	{
		newVerts.push_back(Vector3Transform(vert, transform));
	}
#ifdef VERBOSELOG_COL
	for (auto vert : newVerts)
	{
		std::cout << vert << '\n';
	}
#endif // VERBOSELOG_COL
	newNors.reserve(nors.size());
	for (auto nor : nors)
	{
		newNors.push_back(Vector3Normalize(Vector3Transform(nor, transform)));
	}
	return new MeshCollider(newVerts, newNors);
}

CompoundCollider::CompoundCollider(const vector<Collider*>& cols)
{
	this->colliders = cols;
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
ostream& operator<<(ostream& ostr, Range range)
{
	ostr << range.min << "–" << range.max;
	return ostr;
}
ostream& operator<<(ostream& ostr, Matrix mat)
{
	// TODO: Add overload for quaternion printing.
	Vector3 scale;
	Vector3 pos;
	Quaternion rot;
	MatrixDecompose(mat, &pos, &rot, &scale);
	std::cout << "[p: " << pos << /*", r: " << rot <<*/ ", s: " << scale << "]";
	return ostr;
}
#endif // !NDEBUG

} //namespace phys
