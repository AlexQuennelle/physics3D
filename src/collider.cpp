#include "collider.h"

#include <limits>
#include <optional>
#include <raylib.h>
#include <raymath.h>
#include <vector>
#ifndef NDEBUG
//#define VERBOSELOG_COL
#include <iostream>
#include <ostream>
#ifdef VERBOSELOG_COL
#include "utils.h"
#endif // VERBOSELOG_COL
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
			GetEdgeCrosses(static_cast<MeshCollider*>(collider1),
						   static_cast<MeshCollider*>(collider2), nors);
#ifndef NDEBUG
			for (auto nor : nors)
			{
				DrawLine3D({0.0f, 0.0f, 0.0f}, nor, WHITE);
			}
#endif // !NDEBUG
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
		// NOTE: Debug visualization code
		col1->DebugDraw(trans1, {255, 0, 0, 255});
		col2->DebugDraw(trans2, {255, 0, 0, 255});
		HitObj hitObj{
			.ThisCol = col1, .OtherCol = col2, .HitPos = {0.0f, 0.0f, 0.0f}};
		return hitObj;
	}
	else
	{
		// NOTE: Debug visualization code
		col1->DebugDraw(trans1, {0, 255, 0, 255});
		col2->DebugDraw(trans2, {0, 255, 0, 255});
	}
	return {};
}

void GetEdgeCrosses(const MeshCollider* col1, const MeshCollider* col2,
					vector<Vector3>& out)
{
	vector<Vector3> edges1;
	vector<Vector3> edges2;
	edges1.reserve(col1->edges.size());
	for (auto edge : col1->edges)
	{
		edges1.push_back(
			Vector3Subtract(col1->vertices[edge.a], col1->vertices[edge.b]));
	}
	edges2.reserve(col2->edges.size());
	for (auto edge : col2->edges)
	{
		edges2.push_back(
			Vector3Subtract(col2->vertices[edge.a], col2->vertices[edge.b]));
	}

	for (auto edge1 : edges1)
	{
		for (auto edge2 : edges2)
		{
			out.push_back(Vector3CrossProduct(edge1, edge2));
		}
	}
}

MeshCollider::MeshCollider(const vector<Vector3>& verts,
						   const vector<Edge>& edges,
						   const vector<Vector3>& nors)
{
	this->vertices.insert(this->vertices.end(), verts.begin(), verts.end());
	this->normals.insert(this->normals.end(), nors.begin(), nors.end());
	this->edges.insert(this->edges.end(), edges.begin(), edges.end());
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
		auto newNor = Vector3Transform(nor, trans);
		newNor = Vector3Subtract(newNor, {trans.m12, trans.m13, trans.m14});
		newNors.push_back(Vector3Normalize(newNor));
	}
	vector<Collider*> cols{new MeshCollider(newVerts, this->edges, newNors)};
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
	static const vector<Edge> edges{
		{.a = 0, .b = 1}, {.a = 0, .b = 2}, {.a = 0, .b = 4}, {.a = 3, .b = 4},
		{.a = 1, .b = 5}, {.a = 1, .b = 3}, {.a = 2, .b = 6}, {.a = 2, .b = 5},
		{.a = 3, .b = 7}, {.a = 4, .b = 6}, {.a = 5, .b = 7}, {.a = 6, .b = 7},
	};
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
		auto newNor = Vector3Transform(nor, transform);
		newNor = Vector3Subtract(newNor,
								 {transform.m12, transform.m13, transform.m14});
		newNors.push_back(Vector3Normalize(newNor));
	}
	return new MeshCollider(newVerts, edges, newNors);
}

CompoundCollider::CompoundCollider(const vector<Collider*>& cols)
{
	this->colliders = cols;
}

void CompoundCollider::DebugDraw(const Matrix& transform,
								 const Color& colour) const
{
	for (const auto* col : this->colliders)
	{
		col->DebugDraw(transform, colour);
	}
}
void MeshCollider::DebugDraw(const Matrix& transform, const Color& col) const
{
	for (const auto& edge : this->edges)
	{
		Vector3 start = Vector3Transform(this->vertices[edge.a], transform);
		Vector3 end = Vector3Transform(this->vertices[edge.b], transform);
		DrawLine3D(start, end, col);
	}
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
	Vector3 scale;
	Vector3 pos;
	Quaternion rot;
	MatrixDecompose(mat, &pos, &rot, &scale);
	std::cout << "[p: " << pos << ", r: " << rot << ", s: " << scale << "]";
	return ostr;
}
ostream& operator<<(ostream& ostr, Quaternion quat)
{
	std::cout << QuaternionToEuler(quat);
	return ostr;
}
#endif // !NDEBUG

} //namespace phys
