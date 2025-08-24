#include "collider.h"
#include "halfEdge.h"

#include <cstdint>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <raylib.h>
#include <raymath.h>
#include <utility>
#include <vector>
#ifndef NDEBUG
//#define VERBOSELOG_COL
#include <iostream>
#include <ostream>
#ifdef VERBOSELOG_COL
#include "utils.h"
#endif // VERBOSELOG_COL
#endif // !NDEBUG
#define breakpoint raise(SIGTRAP);

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
	// TODO: Rework for HE structure
	//for (auto edge1 : col1->edges)
	//{
	//	Vector3 axis1 = col1->vertices[edge1.a] - col1->vertices[edge1.b];
	//	for (auto edge2 : col2->edges)
	//	{
	//		Vector3 axis2 = col2->vertices[edge2.a] - col2->vertices[edge2.b];
	//		out.push_back(Vector3Normalize(Vector3CrossProduct(axis1, axis2)));
	//	}
	//}
}

HullCollider::HullCollider(const vector<HE::HVertex>& verts,
						   const vector<HE::FaceInit>& faces,
						   const Vector3 origin)
{
	std::cout << "new hull\n";
	this->origin = origin;
	for (const auto vert : verts)
	{
		this->vertices.push_back(vert);
		this->vertices.back().edgeArr = &this->edges;
	}
	using vertPair = std::pair<uint8_t, uint8_t>;
	std::map<vertPair, HE::HEdge> tmpEdges;
	uint8_t anchor{0};
	for (auto face : faces)
	{
		this->faces.emplace_back(face.normal);
		this->faces.back().edgeArr = &this->edges;
		this->faces.reserve(this->faces.size() + face.indices.size());
		for (int i{0}; i < face.indices.size(); i++)
		{
			vertPair pair{face.indices[i],
						  face.indices[(i + 1) % face.indices.size()]};
			tmpEdges.insert({pair, HE::HEdge()});

			tmpEdges[pair].vertArr = &this->vertices;
			tmpEdges[pair].edgeArr = &this->edges;
			tmpEdges[pair].faceArr = &this->faces;

			tmpEdges[pair].faceID = this->faces.size() - 1;
			tmpEdges[pair].vertID = face.indices[i];
			tmpEdges[pair].nextID = pair.second;
		}
		this->faces.back().edgeID = anchor;
		anchor += face.indices.size();
	}
	for (auto face : faces)
	{
		for (int i{0}; i < face.indices.size(); i++)
		{
			vertPair pair{face.indices[i],
						  face.indices[(i + 1) % face.indices.size()]};
			vertPair pairO{face.indices[(i + 1) % face.indices.size()],
						   face.indices[i]};
			vertPair next{face.indices[(i + 1) % face.indices.size()],
						  face.indices[(i + 2) % face.indices.size()]};
			tmpEdges[pair].nextID =
				std::distance(tmpEdges.begin(), tmpEdges.find(next));
			if (tmpEdges.contains(pairO))
			{
				tmpEdges[pair].twinID =
					std::distance(tmpEdges.begin(), tmpEdges.find(pairO));
				tmpEdges[pairO].twinID =
					std::distance(tmpEdges.begin(), tmpEdges.find(pair));
			}
		}
	}
	this->edges.reserve(tmpEdges.size());
	for (auto& edge : tmpEdges)
	{
		this->edges.push_back(edge.second);
		this->edges.back().Vertex()->edgeID = this->edges.size() - 1;
		this->edges.back().vertID = edge.second.vertID;
		this->edges.back().nextID = edge.second.nextID;
		this->edges.back().vertArr = &this->vertices;
		this->edges.back().edgeArr = &this->edges;
		this->edges.back().faceArr = &this->faces;
	}
}
HullCollider::HullCollider(const HullCollider& copy)
{
	this->vertices.reserve(copy.vertices.size());
	for (auto vert : copy.vertices)
	{
		vert.edgeArr = &this->edges;
		this->vertices.push_back(vert);
	}
	this->edges.reserve(copy.edges.size());
	for (auto edge : copy.edges)
	{
		edge.vertArr = &this->vertices;
		edge.edgeArr = &this->edges;
		edge.faceArr = &this->faces;
		this->edges.push_back(edge);
	}
	this->faces.reserve(copy.faces.size());
	for (auto face : copy.faces)
	{
		face.edgeArr = &this->edges;
		this->faces.push_back(face);
	}
}
void HullCollider::GetTransformed(const Matrix trans,
								  vector<Col_Sptr>& out) const
{
	auto newCol = std::make_shared<HullCollider>(HullCollider(*this));
	for (int i{0}; i < newCol->vertices.size(); i++)
	{
		newCol->vertices[i] = newCol->vertices[i] * trans;
	}
	for (int i{0}; i < newCol->faces.size(); i++)
	{
		auto newNor = newCol->faces[i].normal * trans;
		newCol->faces[i].normal = Vector3Normalize(
			Vector3Subtract(newNor, {trans.m12, trans.m13, trans.m14}));
	}
	newCol->origin = newCol->origin * trans;
	out.push_back(newCol);
}
void HullCollider::GetNormals(vector<Vector3>& out) const
{
	for (const auto face : this->faces)
	{
		out.push_back(face.normal);
	}
}
Range HullCollider::GetProjection(const Vector3 nor) const
{
	Range proj{
		.min = std::numeric_limits<float>::max(),
		.max = std::numeric_limits<float>::min(),
	};
	for (const auto vert : this->vertices)
	{
		float projected = Vector3DotProduct(vert.Vec(), nor);
		proj.min = projected < proj.min ? projected : proj.min;
		proj.max = projected > proj.max ? projected : proj.max;
	}
	return proj;
}
Vector3 HullCollider::GetSupportPoint(const Vector3& axis) const
{
	// TODO: Potentially clean up
	Vector3 support{};
	float supportVal{-1.0f};
	for (const auto vert : this->vertices)
	{
		if (Vector3DotProduct(
				axis, Vector3Normalize(vert.Vec() - this->origin)) > supportVal)
		{
			supportVal = Vector3DotProduct(
				axis, Vector3Normalize(vert.Vec() - this->origin));
			support = vert.Vec();
		}
	}
	return support;
}
void HullCollider::DebugDraw(const Matrix& transform, const Color& col) const
{
	for (const auto& edge : this->edges)
	{
		Vector3 start = edge.Vertex()->Vec() * transform;
		Vector3 end = edge.Twin()->Vertex()->Vec() * transform;
		DrawLine3D(start, end, col);
	}
	DrawSphere(this->origin * transform, 0.025f, col);
}
void HullCollider::GetFaceInits(vector<HE::FaceInit>& out) {}

std::shared_ptr<HullCollider> CreateBoxCollider(Matrix transform)
{
#ifdef VERBOSELOG_COL
	std::cout << transform << '\n';
#endif
	static const vector<HE::HVertex> verts{
		{.x = -0.5f, .y = -0.5f, .z = -0.5f},
		{.x = 0.5f, .y = -0.5f, .z = -0.5f},
		{.x = -0.5f, .y = 0.5f, .z = -0.5f},
		{.x = 0.5f, .y = 0.5f, .z = -0.5f},

		{.x = -0.5f, .y = -0.5f, .z = 0.5f},
		{.x = 0.5f, .y = -0.5f, .z = 0.5f},
		{.x = -0.5f, .y = 0.5f, .z = 0.5f},
		{.x = 0.5f, .y = 0.5f, .z = 0.5f},
	};
	vector<HE::HVertex> newVerts;
	static const vector<Vector3> nors{
		{.x = 1.0f, .y = 0.0f, .z = 0.0f},
		{.x = -1.0f, .y = 0.0f, .z = 0.0f},
		{.x = 0.0f, .y = 1.0f, .z = 0.0f},
		{.x = 0.0f, .y = -1.0f, .z = 0.0f},
		{.x = 0.0f, .y = 0.0f, .z = 1.0f},
		{.x = 0.0f, .y = 0.0f, .z = -1.0f},
	};

	newVerts.reserve(verts.size());
	for (auto vert : verts)
	{
		newVerts.push_back(vert * transform);
	}
#ifdef VERBOSELOG_COL
	for (auto vert : newVerts)
	{
		std::cout << vert << '\n';
	}
#endif // VERBOSELOG_COL
	vector<HE::FaceInit> faces;
	faces.reserve(nors.size());
	for (auto nor : nors)
	{
		auto newNor = nor * transform;
		newNor = Vector3Subtract(newNor,
								 {transform.m12, transform.m13, transform.m14});
		faces.emplace_back(Vector3Normalize(newNor));
	}
	faces[0].indices = {1, 5, 7, 3};
	faces[1].indices = {0, 2, 6, 4};
	faces[2].indices = {4, 5, 1, 0};
	faces[3].indices = {3, 7, 6, 2};
	faces[4].indices = {0, 1, 3, 2};
	faces[5].indices = {4, 6, 7, 5};
	return std::make_shared<HullCollider>(newVerts, faces,
										  Vector3Zero() * transform);
}

CompoundCollider::CompoundCollider(const vector<Col_Sptr>& cols)
{
	this->colliders = cols;
}
void CompoundCollider::GetTransformed(const Matrix trans,
									  vector<Col_Sptr>& out) const
{
	for (const auto& elem : this->colliders)
	{
		elem->GetTransformed(trans, out);
	}
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
