#include "collider.h"

#include <limits>
#include <memory>
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

void GetEdgeCrosses(const std::shared_ptr<HullCollider> col1,
					const std::shared_ptr<HullCollider> col2,
					vector<Vector3>& out)
{
	for (auto edge1 : col1->edges)
	{
		Vector3 axis1 = col1->vertices[edge1.a] - col1->vertices[edge1.b];
		for (auto edge2 : col2->edges)
		{
			Vector3 axis2 = col2->vertices[edge2.a] - col2->vertices[edge2.b];
			out.push_back(Vector3Normalize(Vector3CrossProduct(axis1, axis2)));
		}
	}
}

HullCollider::HullCollider(const vector<Vector3>& verts,
						   const vector<Edge>& edges,
						   const vector<Vector3>& nors)
{
	this->vertices.insert(this->vertices.end(), verts.begin(), verts.end());
	this->normals.insert(this->normals.end(), nors.begin(), nors.end());
	this->edges.insert(this->edges.end(), edges.begin(), edges.end());
}
HullCollider::HullCollider(const Vector3 origin, const vector<Vector3>& verts,
						   const vector<Edge>& edges,
						   const vector<Vector3>& nors)
{
	this->vertices.insert(this->vertices.end(), verts.begin(), verts.end());
	this->normals.insert(this->normals.end(), nors.begin(), nors.end());
	this->edges.insert(this->edges.end(), edges.begin(), edges.end());
	this->origin = origin;
}
vector<Col_Sptr> HullCollider::GetTransformed(const Matrix trans) const
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
	vector<Col_Sptr> cols{
		Col_Sptr(new HullCollider(Vector3Transform(this->origin, trans),
								  newVerts, this->edges, newNors))};
	return cols;
}
void HullCollider::GetNormals(vector<Vector3>& out) const
{
	out.insert(out.end(), this->normals.begin(), this->normals.end());
}
Range HullCollider::GetProjection(const Vector3 nor) const
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
Vector3 HullCollider::GetSupportPoint(const Vector3& axis) const
{
	Vector3 support{};
	float supportVal{-1.0f};
	for (const auto vert : this->vertices)
	{
		if (Vector3DotProduct(axis, Vector3Normalize(vert - this->origin)) >
			supportVal)
		{
			supportVal =
				Vector3DotProduct(axis, Vector3Normalize(vert - this->origin));
			support = vert;
		}
	}
	return support;
}
void HullCollider::DebugDraw(const Matrix& transform, const Color& col) const
{
	for (const auto& edge : this->edges)
	{
		Vector3 start = Vector3Transform(this->vertices[edge.a], transform);
		Vector3 end = Vector3Transform(this->vertices[edge.b], transform);
		DrawLine3D(start, end, col);
	}
	DrawSphere(this->origin * transform, 0.025f, col);
}

std::shared_ptr<HullCollider> CreateBoxCollider(Matrix transform)
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
		{.x = 1.0f, .y = 0.0f, .z = 0.0f},	{.x = 0.0f, .y = 1.0f, .z = 0.0f},
		{.x = 0.0f, .y = 0.0f, .z = 1.0f},	{.x = -1.0f, .y = 0.0f, .z = 0.0f},
		{.x = 0.0f, .y = -1.0f, .z = 0.0f}, {.x = 0.0f, .y = 0.0f, .z = -1.0f},
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
	return std::make_shared<HullCollider>(Vector3Zero() * transform, newVerts,
										  edges, newNors);
}

CompoundCollider::CompoundCollider(const vector<Col_Sptr>& cols)
{
	this->colliders = cols;
}
vector<Col_Sptr> CompoundCollider::GetTransformed(const Matrix trans) const
{
	vector<Col_Sptr> cols;
	for (const auto& elem : this->colliders)
	{
		vector<Col_Sptr> transformed = elem->GetTransformed(trans);
		cols.insert(cols.end(), transformed.begin(), transformed.end());
	}
	return cols;
}
void CompoundCollider::GetNormals(vector<Vector3>& out) const
{
	for (const auto& col : this->colliders)
	{
		col->GetNormals(out);
	}
}
Vector3 CompoundCollider::GetSupportPoint(const Vector3& axis) const
{
	return {0.0f, 0.0f, 0.0f};
}
void CompoundCollider::DebugDraw(const Matrix& transform,
								 const Color& colour) const
{
	for (const auto& col : this->colliders)
	{
		col->DebugDraw(transform, colour);
	}
}

#ifndef NDEBUG
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
