#include "collider.h"
#include "halfEdge.h"
#include "utils.h"

#include <cassert>
#include <cstdint>
#include <iterator>
#include <limits>
#include <map>
#include <ranges>
#include <raylib.h>
#include <raymath.h>
#include <set>
#include <utility>
#include <variant>
#include <vector>
#ifndef NDEBUG
#include <iostream>
//#define VERBOSELOG_COL
#ifdef VERBOSELOG_COL
#include "utils.h"
#endif // VERBOSELOG_COL
#endif // !NDEBUG

namespace phys
{

namespace r = std::ranges;
namespace rv = std::ranges::views;

using std::vector;
#ifndef NDEBUG
using std::ostream;
#endif

auto GetEdgeCrosses(const HullCollider& col1, const HullCollider& col2)
	-> vector<Vector3Tuple>
{
	std::cout << '\n';
	auto crosses = [](auto pair) -> Vector3Tuple
	{
		auto cross = Vector3Normalize(
			Vector3CrossProduct(std::get<0>(pair), std::get<1>(pair)));
		return {std::get<0>(pair), std::get<1>(pair), cross};
	};
	auto deDupe = [](auto pair) -> bool
	{
		return !Vector3Equivalent(std::get<0>(pair), std::get<1>(pair));
	};
	return rv::cartesian_product(col1.edgeDirs, col2.edgeDirs)
		   | rv::filter(deDupe)
		   | rv::transform(crosses)
		   | r::to<std::vector<Vector3Tuple>>();
}
auto GetClosestPoints(const EdgeHit hit) -> std::pair<Vector3, Vector3>
{
	auto cross1 = Vector3CrossProduct(hit.direction1, hit.normal);
	auto cross2 = Vector3CrossProduct(hit.direction2, hit.normal);
	auto point1 = hit.support1
				  + Vector3Scale(
					  hit.direction1,
					  (Vector3DotProduct((hit.support2 - hit.support1), cross2)
					   / Vector3DotProduct(hit.direction1, cross2)));
	auto point2 = hit.support2
				  + Vector3Scale(
					  hit.direction2,
					  (Vector3DotProduct((hit.support1 - hit.support2), cross1)
					   / Vector3DotProduct(hit.direction2, cross1)));
	return {point1, point2};
}

HullCollider::HullCollider(const vector<HE::HVertex>& verts,
						   const vector<HE::FaceInit>& faces,
						   const Vector3 origin) : origin(origin)
{
#ifndef NDEBUG
	// std::cout << "new hull\n";
#endif // NDEBUG

	auto initVerts = [this](auto vert) -> auto
	{
		vert.edgeArr = &this->edges;
		return vert;
	};
	this->vertices
		= verts | rv::transform(initVerts) | r::to<vector<HE::HVertex>>();

	using VertPair = std::pair<uint8_t, uint8_t>;
	std::map<VertPair, HE::HEdge> tmpEdges;
	std::set<VertPair> uniqueEdges;
	vector<VertPair> anchors;
	anchors.resize(faces.size());
	this->faces.reserve(faces.size());
	for (auto face : faces)
	{
		this->faces.emplace_back(face.normal);
		this->faces.back().edgeArr = &this->edges;
		for (uint64_t i{0}; i < face.indices.size(); i++)
		{
			VertPair pair{face.indices[i],
						  face.indices[(i + 1) % face.indices.size()]};
			tmpEdges.insert({pair, HE::HEdge()});

			tmpEdges[pair].vertArr = &this->vertices;
			tmpEdges[pair].edgeArr = &this->edges;
			tmpEdges[pair].faceArr = &this->faces;

			tmpEdges[pair].faceID
				= static_cast<uint8_t>(this->faces.size() - 1);
			tmpEdges[pair].vertID = face.indices[i];
			tmpEdges[pair].nextID = pair.second;
			anchors[this->faces.size() - 1] = pair;

			if (!uniqueEdges.contains(pair)
				&& !uniqueEdges.contains({pair.second, pair.first}))
			{
				uniqueEdges.emplace(pair);
			}
		}
	}
	for (uint64_t i{0}; i < this->faces.size(); i++)
	{
		auto face = faces[i];
		this->faces[i].edgeID = static_cast<uint8_t>(
			std::distance(tmpEdges.begin(), tmpEdges.find(anchors[i])));
		for (uint64_t j{0}; j < face.indices.size(); j++)
		{
			VertPair pair{face.indices[j],
						  face.indices[(j + 1) % face.indices.size()]};
			VertPair pairO{face.indices[(j + 1) % face.indices.size()],
						   face.indices[j]};
			VertPair next{face.indices[(j + 1) % face.indices.size()],
						  face.indices[(j + 2) % face.indices.size()]};
			tmpEdges[pair].nextID = static_cast<uint8_t>(
				std::distance(tmpEdges.begin(), tmpEdges.find(next)));
			if (tmpEdges.contains(pairO))
			{
				tmpEdges[pair].twinID = static_cast<uint8_t>(
					std::distance(tmpEdges.begin(), tmpEdges.find(pairO)));
				tmpEdges[pairO].twinID = static_cast<uint8_t>(
					std::distance(tmpEdges.begin(), tmpEdges.find(pair)));
			}
		}
	}

	auto getEdgeDir = [this](auto pair) -> Vector3
	{
		return this->vertices[pair.first].Vec()
			   - this->vertices[pair.second].Vec();
	};
	this->edgeDirs = uniqueEdges
					 | rv::transform(getEdgeDir)
					 | rv::transform(Vector3Normalize)
					 | r::to<vector<Vector3>>();

	auto initEdge = [this](HE::HEdge edge) -> HE::HEdge
	{
		edge.Vertex()->edgeID = static_cast<uint8_t>(this->edges.size() - 1);
		edge.vertArr = &this->vertices;
		edge.edgeArr = &this->edges;
		edge.faceArr = &this->faces;
		return edge;
	};
	this->edges = tmpEdges
				  | rv::transform([](auto edge) -> auto { return edge.second; })
				  | rv::transform(initEdge)
				  | r::to<vector<HE::HEdge>>();
}
HullCollider::HullCollider(const HullCollider& copy) : edgeDirs(copy.edgeDirs)
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
								  vector<Collider>& out) const
{
	HullCollider newCol{HullCollider(*this)};
	for (uint64_t i{0}; i < newCol.vertices.size(); i++)
	{
		newCol.vertices[i] = newCol.vertices[i] * trans;
	}
	for (uint64_t i{0}; i < newCol.faces.size(); i++)
	{
		auto newNor = newCol.faces[i].normal * trans;
		newCol.faces[i].normal = Vector3Normalize(
			Vector3Subtract(newNor, {trans.m12, trans.m13, trans.m14}));
	}
	for (auto& dir : newCol.edgeDirs)
	{
		Vector3 translation;
		Quaternion rotation;
		Vector3 scale;
		MatrixDecompose(trans, &translation, &rotation, &scale);
		dir = dir * QuaternionToMatrix(rotation);
	}
	newCol.origin = newCol.origin * trans;
	out.emplace_back(newCol);
}
void HullCollider::GetNormals(vector<Vector3>& out) const
{
	for (const auto face : this->faces)
	{
		out.push_back(face.normal);
	}
}
auto HullCollider::GetProjection(const Vector3 nor) const -> Range
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
auto HullCollider::GetSupportPoint(const Vector3 axis) const -> Vector3
{
	// NOTE: Potentially clean up
	Vector3 support{};
	float supportVal{-1.0f};
	for (const auto vert : this->vertices)
	{
		if (Vector3DotProduct(axis, (vert.Vec() - this->origin)) > supportVal)
		{
			supportVal = Vector3DotProduct(axis, (vert.Vec() - this->origin));
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
	for (const auto& face : this->faces)
	{
		for (const auto& edge : face)
		{
			Vector3 start = edge.Twin()->Vertex()->Vec() * transform;
			Vector3 end
				= (Vector3RotateByAxisAngle(Vector3Negate(edge.Dir()) * 0.1f,
											face.normal, 20.0f * DEG2RAD)
				   + edge.Twin()->Vertex()->Vec())
				  * transform;
			DrawLine3D(start, end, col);
		}
		DrawLine3D(face.Center() * transform,
				   (face.Center() + (face.normal * 0.1f)) * transform, col);
	}
	DrawSphere(this->origin * transform, 0.025f, col);
}
void HullCollider::DebugDrawEdge(const uint64_t index) const
{
	DrawSphere(this->edges[index].Vertex()->Vec(), 0.05f, GREEN);
	DrawSphere(this->edges[index].Next()->Vertex()->Vec(), 0.05f, GREEN);
}

auto CreateBoxCollider(Matrix transform) -> Collider
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
		{.x = 1.0f, .y = 0.0f, .z = 0.0f}, {.x = -1.0f, .y = 0.0f, .z = 0.0f},
		{.x = 0.0f, .y = 1.0f, .z = 0.0f}, {.x = 0.0f, .y = -1.0f, .z = 0.0f},
		{.x = 0.0f, .y = 0.0f, .z = 1.0f}, {.x = 0.0f, .y = 0.0f, .z = -1.0f},
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
	faces[2].indices = {3, 7, 6, 2};
	faces[3].indices = {4, 5, 1, 0};
	faces[4].indices = {4, 6, 7, 5};
	faces[5].indices = {0, 1, 3, 2};
	// return std::make_shared<HullCollider>(newVerts, faces,
	// 									  Vector3Zero() * transform);
	HullCollider newCol{newVerts, faces, Vector3Zero() * transform};
	return {newCol};
}

CompoundCollider::CompoundCollider(const vector<Collider>& cols) :
	colliders(cols)
{ }
void CompoundCollider::GetTransformed(const Matrix trans,
									  vector<Collider>& out) const
{
	for (const auto& elem : this->colliders)
	{
		// elem->GetTransformed(trans, out);
		std::visit([trans, &out](const isCollider auto& col) -> void
				   { col.GetTransformed(trans, out); }, elem);
	}
}
void CompoundCollider::GetNormals(vector<Vector3>& out) const
{
	for (const auto& col : this->colliders)
	{
		// col->GetNormals(out);
		std::visit([&out](const isCollider auto& col) -> void
				   { col.GetNormals(out); }, col);
	}
}
auto CompoundCollider::GetSupportPoint(const Vector3 /*axis*/) -> Vector3
{
	return {0.0f, 0.0f, 0.0f};
}
void CompoundCollider::DebugDraw(const Matrix& transform,
								 const Color& colour) const
{
	for (const auto& col : this->colliders)
	{
		// col->DebugDraw(transform, colour);
		std::visit([transform, colour](const isCollider auto& col) -> void
				   { col.DebugDraw(transform, colour); }, col);
	}
}

#ifndef NDEBUG
auto operator<<(ostream& ostr, Vector3 vec) -> ostream&
{
	ostr << '(' << vec.x << ", " << vec.y << ", " << vec.z << ')';
	return ostr;
}
auto operator<<(ostream& ostr, Range range) -> ostream&
{
	ostr << range.min << "–" << range.max;
	return ostr;
}
auto operator<<(ostream& ostr, Matrix mat) -> ostream&
{
	Vector3 scale;
	Vector3 pos;
	Quaternion rot;
	MatrixDecompose(mat, &pos, &rot, &scale);
	std::cout << "[p: " << pos << ", r: " << rot << ", s: " << scale << "]";
	return ostr;
}
auto operator<<(ostream& ostr, Quaternion quat) -> ostream&
{
	std::cout << QuaternionToEuler(quat);
	return ostr;
}
#endif // !NDEBUG

} //namespace phys
