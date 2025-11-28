#include "physObject.h"
#include "collider.h"
#include "halfEdge.h"
#include "utils.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <optional>
#include <ranges>
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
#include <variant>

namespace phys
{

using std::optional;

namespace r = std::ranges;
namespace rv = std::views;

/** @brief Tests if a 3D point planar to a face lies within the polygon
 *         described by its edges.
 */
auto IsPointInPoly3D(const Vector3 point, const HE::HFace& poly) -> bool;

auto GenFaceContact(const HE::HFace& ref, const HE::HFace& incident)
	-> vector<Vector3>;

auto CheckCollision(const PhysObject& obj1, const PhysObject& obj2)
	-> optional<HitObj>
{
	vector<Collider> cols1;
	obj1.GetColliderT(MatrixIdentity(), cols1);
	vector<Collider> cols2;
	obj2.GetColliderT(obj2.GetTransformM() * MatrixInvert(obj1.GetTransformM()),
					  cols2);
	bool collision = false;
	for (const auto& col1 : cols1)
	{
		for (const auto& col2 : cols2)
		{
			auto faces1 = CheckFaceNors(col1, col2);
			if (faces1.penetration <= 0)
				continue;
			auto faces2 = CheckFaceNors(col2, col1);
			if (faces2.penetration <= 0)
				continue;
			auto edges = CheckEdgeNors(col1, col2);
			if (edges.penetration <= 0)
				continue;

			bool isEdgeCol{
				(edges.penetration < faces1.penetration)
					&& (edges.penetration < faces2.penetration),
			};
			if (isEdgeCol)
			{
				// TODO: Get rid of this and rework hit object
				std::cout << "Edge Collision\n";
				auto [closest1, closest2] = GetClosestPoints(edges);
				auto hitPos
					= closest1 + (edges.normal * (edges.penetration / 2.0f));
#ifndef NDEBUG
				DrawSphere(edges.support1, 0.01f, BLUE);
				DrawSphere(edges.twin1, 0.01f, BLUE);
				DrawSphere(edges.support2, 0.01f, BLUE);
				DrawSphere(edges.twin2, 0.01f, BLUE);

				DrawSphere(hitPos, 0.025f, BLUE);
				DrawSphere(closest1, 0.01f, BLUE);
				DrawSphere(closest2, 0.01f, BLUE);
				DrawLine3D(closest1, closest2, BLUE);
#endif // !NDEBUG
			}
			else
			{
				std::cout << "Face Collision\n";
				CheckFaceCollision(col1, col2, faces1, faces2);
			}
			collision |= true;
		}
	}
#ifndef NDEBUG
	//Draw both objects' colliders in the local coordinate space of object 1
	std::visit([](const isCollider auto& col) -> void
			   { col.DebugDraw(MatrixIdentity(), {255, 255, 255, 255}); },
			   obj1.GetCollider());
	std::visit(
		[&obj1, &obj2](const isCollider auto& col) -> auto
		{
			col.DebugDraw(obj2.GetTransformM()
							  * MatrixInvert(obj1.GetTransformM()),
						  {255, 255, 255, 255});
		},
		obj2.GetCollider());
#endif // !NDEBUG
	if (collision)
	{
		// NOTE: Debug visualization code
		std::visit([&obj1](const isCollider auto& col) -> auto
				   { col.DebugDraw(obj1.GetTransformM(), {255, 0, 0, 255}); },
				   obj1.GetCollider());
		std::visit([&obj2](const isCollider auto& col) -> auto
				   { col.DebugDraw(obj2.GetTransformM(), {255, 0, 0, 255}); },
				   obj2.GetCollider());
		HitObj hitObj{
			.HitPos = {0.0f, 0.0f, 0.0f}, .ThisCol = &obj1, .OtherCol = &obj2};
		return hitObj;
	}
	else
	{
		// NOTE: Debug visualization code
		std::visit([&obj1](const isCollider auto& col) -> auto
				   { col.DebugDraw(obj1.GetTransformM(), {0, 255, 0, 255}); },
				   obj1.GetCollider());
		std::visit([&obj2](const isCollider auto& col) -> auto
				   { col.DebugDraw(obj2.GetTransformM(), {0, 255, 0, 255}); },
				   obj2.GetCollider());
	}
	return {};
}
void CheckFaceCollision(const Collider& colA, const Collider& colB,
						const FaceHit faces1, const FaceHit faces2)
{
	// Face collision
	const auto& hull1{std::get<0>(colA)};
	const auto& hull2{std::get<0>(colB)};
	if (faces1.penetration < faces2.penetration)
	{
		const auto& ref = hull1.GetFace(faces1.id);
		float dot{1.0f};
		uint32_t incidentID{0};
		for (uint32_t i{0}; i < hull2.FaceCount(); i++)
		{
			const auto& face{hull2.GetFace(i)};
			if (float newDot{Vector3DotProduct(face.normal, ref.normal)};
				newDot < dot)
			{
				dot = newDot;
				incidentID = i;
			}
		}
		GenFaceContact(ref, hull2.GetFace(incidentID));
		DrawLine3D(faces1.support,
				   faces1.support
					   + (hull1.GetFace(faces1.id).normal * faces1.penetration),
				   RED);
	}
	else
	{
		const auto& ref = hull2.GetFace(faces2.id);
		float dot{1.0f};
		uint32_t incidentID{0};
		for (uint32_t i{0}; i < hull1.FaceCount(); i++)
		{
			const auto& face{hull1.GetFace(i)};
			if (float newDot{Vector3DotProduct(face.normal, ref.normal)};
				newDot < dot)
			{
				dot = newDot;
				incidentID = i;
			}
		}
		GenFaceContact(ref, hull1.GetFace(incidentID));
		DrawLine3D(faces2.support,
				   faces2.support
					   + (hull2.GetFace(faces2.id).normal * faces2.penetration),
				   RED);
	}
	// return true;
}
auto GenFaceContact(const HE::HFace& ref, const HE::HFace& incident)
	-> vector<Vector3>
{
	vector<HE::HEdge> surface{};
	vector<HE::HVertex> sVerts{};
	for (const auto& edge : incident)
	{
		sVerts.push_back(*edge.Vertex());
		HE::HEdge newEdge{
			.vertID = static_cast<uint8_t>(sVerts.size() - 1),
			.twinID = 0,
			.nextID = static_cast<uint8_t>(surface.size() + 1),
			.faceID = 0,
			.vertArr = &sVerts,
			.edgeArr = &surface,
			.faceArr = nullptr,
		};
		surface.push_back(newEdge);
	}
	surface[surface.size() - 1].nextID = 0; // Close the loop

	for (auto& edgeRef : *ref.Edge())
	{
		const Vector3 planeNor{
			Vector3Normalize(Vector3CrossProduct(edgeRef.Dir(), ref.normal))};

		auto sideTest = [edgeRef, planeNor](Vector3 point) -> bool
		{
			return HE::IsPointBehindPlane(
				{.pos = edgeRef.Vertex()->Vec(), .nor = planeNor}, point);
		};

		vector<HE::HEdge> newSurface{0};
		vector<HE::HVertex> newVerts{0};
		for (const auto& sEdge : *surface.data())
		{
			Vector3 edgeDir{sEdge.Dir()};
			Vector3 edgeVert = sEdge.Vertex()->Vec();

			bool bothInside{false};
			if (Vector3DotProduct(planeNor, edgeDir) >= 0)
			{
				edgeVert = sEdge.Next()->Vertex()->Vec();
				edgeDir = -edgeDir;

				bothInside = sideTest(sEdge.Next()->Vertex()->Vec())
							 && !sideTest(sEdge.Vertex()->Vec());
			}
			float dist{
				Vector3DotProduct(edgeRef.Vertex()->Vec() - edgeVert, planeNor)
				/ Vector3DotProduct(planeNor, edgeDir)};

			if (std::isinf(dist))
			{
				// TODO: Potentially find a more elegant solution
				//       It's close enough for now, but not perfect
				auto newPos{sEdge.Next()->Vertex()->Vec()};
				bothInside = false;
				if (!sideTest(newPos))
				{
					newPos = newPos
							 + (-planeNor)
							 * Vector3DotProduct(
								 planeNor, newPos - edgeRef.Vertex()->Vec());
				}
				newVerts.emplace_back(newPos.x, newPos.y, newPos.z,
									  newVerts.size());
			}
			else if (dist >= sEdge.Length())
			{
				auto newPos{sEdge.Next()->Vertex()->Vec()};
				newVerts.emplace_back(newPos.x, newPos.y, newPos.z,
									  newVerts.size());
			}
			else if (dist >= 0)
			{
				Vector3 newPos{edgeVert + (edgeDir * dist * 0.999f)};
				newVerts.emplace_back(newPos.x, newPos.y, newPos.z,
									  newSurface.size());
			}
			if (bothInside)
			{
				auto newPos{sEdge.Next()->Vertex()->Vec()};
				newVerts.emplace_back(newPos.x, newPos.y, newPos.z,
									  newVerts.size());
			}
		}
		for ([[maybe_unused]] const auto point : newVerts)
		{
			newSurface.emplace_back(static_cast<uint8_t>(newSurface.size()), 0,
									static_cast<uint8_t>(newSurface.size() + 1),
									0, &sVerts, &surface, nullptr);
		}
		newSurface[newSurface.size() - 1].nextID = 0; // Close the loop
		sVerts.swap(newVerts);
		surface.swap(newSurface);
	}
#ifndef NDEBUG
	for (auto& edge : surface)
	{
		DrawLine3D(edge.Vertex()->Vec(), edge.Next()->Vertex()->Vec(), RED);
		auto start = edge.Next()->Vertex()->Vec();
		auto end = (Vector3RotateByAxisAngle((-edge.Dir() * 0.1f), -ref.normal,
											 20.0f * DEG2RAD)
					+ start);
		DrawLine3D(start, end, RED);
	}
#endif // !NDEBUG
	auto filter = [&ref](const HE::HVertex vert) -> bool
	{
		return !HE::IsPointBehindPlane(ref.Plane(), vert.Vec());
	};
	auto transform = [&ref](const HE::HVertex vert) -> Vector3
	{
		auto refPoint = ref.Edge()->Vertex()->Vec();
		auto nor = ref.normal;
		return vert.Vec()
			   + (-nor)
			   * Vector3DotProduct(nor, vert.Vec() - refPoint);
	};
	vector<Vector3> contact = sVerts
							  | rv::filter(filter)
							  | rv::transform(transform)
							  | std::ranges::to<vector>();
#ifndef NDEBUG
	for (const auto point : contact)
	{
		DrawSphere(point, 0.01f, RED);
	}
#endif // !NDEBUG
	return contact;
}
auto CheckRaycast(const Ray ray, PhysObject& obj) -> std::optional<RaycastHit>
{
	vector<Collider> colliders;
	std::visit([&obj, &colliders](const isCollider auto& col) -> auto
			   { col.GetTransformed(obj.GetTransformM(), colliders); },
			   obj.GetCollider());
	RaycastHit hitObj{.hitDist = std::numeric_limits<float>::max(),
					  .hitPos = Vector3Zero(),
					  .hitObj = &obj};
	bool isHit{false};
	for (const auto& collider : colliders)
	{
		const HullCollider& hull{std::get<0>(collider)};
		for (uint32_t i{0}; i < hull.FaceCount(); i++)
		{
			const auto& face = hull.GetFace(i);
			if (Vector3DotProduct(face.normal, ray.direction) > 0)
				continue;
			float dist
				= Vector3DotProduct(face.Edge()->Vertex()->Vec() - ray.position,
									face.normal)
				  / Vector3DotProduct(face.normal, ray.direction);
			if (dist >= 0)
			{
				auto hitPos = ray.position + (ray.direction * dist);
				if (IsPointInPoly3D(hitPos, face))
				{
					isHit |= true;
					if (dist < hitObj.hitDist)
					{
						hitObj.hitDist = dist;
						hitObj.hitPos = hitPos;
					}
				}
			}
			else
			{
				continue;
			}
		}
	}

	if (isHit)
	{
		return hitObj;
	}
	else
	{
		return {};
	}
}
auto IsPointInPoly3D(const Vector3 point, const HE::HFace& poly) -> bool
{
	bool inPoly{false};
	Vector3 xAxis = poly.Edge()->Dir();
	Vector3 yAxis = (Vector3CrossProduct(poly.Edge()->Dir(), poly.normal));
	Vector2 point2D
		= {Vector3DotProduct(point, xAxis), Vector3DotProduct(point, yAxis)};
	for (const auto& edge : poly)
	{
		Vector2 point1 = {Vector3DotProduct(edge.Vertex()->Vec(), xAxis),
						  Vector3DotProduct(edge.Vertex()->Vec(), yAxis)};
		point1 = point1 - point2D;
		Vector2 point2
			= {Vector3DotProduct(edge.Next()->Vertex()->Vec(), xAxis),
			   Vector3DotProduct(edge.Next()->Vertex()->Vec(), yAxis)};
		point2 = point2 - point2D;

		if (point1.x < 0 && point2.x < 0)
			continue;

		bool edgeCross{false};
		if ((point1.y > 0 && point2.y <= 0) || (point2.y > 0 && point1.y <= 0))
		{
			if (point1.x == point2.x)
				edgeCross = true;
			else
			{
				float slope = (point1.y - point2.y) / (point1.x - point2.x);
				edgeCross = -(point1.y / slope) + point1.x >= 0;
			}
		}
		inPoly = inPoly ^ edgeCross;
	}
	return inPoly;
}
auto CheckFaceNors(Collider colA, Collider colB) -> FaceHit
{
#ifndef NDEBUG
	// vector<Vector3> nors;
	// std::visit([&nors](const isCollider auto& col) -> void
	// {
	// 	col.GetNormals(nors);
	// }, colA);
	// srand(static_cast<int>(nors[0].x + nors[1].y));
#endif // !NDEBUG
	FaceHit hit{};
	hit.penetration = std::numeric_limits<float>::max();
	const HullCollider& hull1 = std::get<0>(colA);
	for (uint32_t i{0}; i < hull1.faces.size(); i++)
	{
		Vector3 nor = hull1.faces[i].normal;
		Vector3 support{std::visit([nor](const isCollider auto& col) -> Vector3
								   { return col.GetSupportPoint(-nor); },
								   colB)};
		float penetration
			= Vector3DotProduct(hull1.faces[i].Edge()->Vertex()->Vec(), nor)
			  - Vector3DotProduct(support, nor);
		if (penetration < hit.penetration)
		{
			hit.penetration = penetration;
			hit.id = i;
			hit.support = support;
		}
		//DrawLine3D(support, support + (nor * penertration), RED);
#ifndef NDEBUG
		// std::visit([support, nor](const isCollider auto& col) -> void
		// {
		// 	Color color = {
		// 		static_cast<uint8_t>(rand()),
		// 		static_cast<uint8_t>(rand()),
		// 		static_cast<uint8_t>(rand()),
		// 		255,
		// 	};
		// 	DrawLine3D(col.GetOrigin(), col.GetOrigin() + nor, color);
		// 	DrawSphere(col.GetOrigin() + nor, 0.025f, color);
		// 	DrawSphere(support, 0.05f, color);
		// }, colA);
#endif // !NDEBUG
	}
	return hit;
}
auto CheckEdgeNors(Collider colA, Collider colB) -> EdgeHit
{
	const HullCollider& hull1 = std::get<0>(colA);
	const HullCollider& hull2 = std::get<0>(colB);

	auto normalizeDirs = [&hull2, &hull1](auto nor) -> Vector3Tuple
	{
		auto dir = hull2.origin - hull1.origin;
		if (Vector3DotProduct(std::get<2>(nor), dir) >= 0)
			std::get<2>(nor) = -std::get<2>(nor);
		return {std::get<0>(nor), std::get<1>(nor), std::get<2>(nor)};
	};
	auto genHitObject = [&hull1, &hull2](auto triple) -> EdgeHit
	{
		Vector3 normal = std::get<2>(triple);
		Vector3 dir1 = std::get<1>(triple);
		Vector3 dir2 = std::get<0>(triple);
		auto [support1, twin1] = hull2.GetSupportPoints(-normal, dir1);
		auto [support2, twin2] = hull1.GetSupportPoints(normal, dir2);
		return {
			.penetration = 0.0f,
			.support1 = support1,
			.twin1 = twin1,
			.direction1 = dir1,
			.support2 = support2,
			.twin2 = twin2,
			.direction2 = dir2,
			.normal = normal,
		};
	};
	auto checkBounds = [](auto hit) -> bool
	{
		auto [closest1, closest2] = GetClosestPoints(hit);
		return IsPointOnSegment(hit.support1, hit.twin1, closest1)
			   && IsPointOnSegment(hit.support2, hit.twin2, closest2);
	};
	auto getPenetration = [&hull1](auto hit) -> EdgeHit
	{
		hit.penetration = hull1.GetProjection(hit.normal).max
						  - Vector3DotProduct(hit.normal, hit.support1);
		return hit;
	};
	auto nors = GetEdgeCrosses(hull1, hull2)
				| rv::transform(normalizeDirs)
				| rv::transform(genHitObject)
				| rv::filter(checkBounds)
				| rv::transform(getPenetration)
				| r::to<vector<EdgeHit>>();

	if (nors.empty())
		return {};

	return *r::min_element(nors, {}, &EdgeHit::penetration);
}

PhysObject::PhysObject(const Vector3 pos, const Mesh mesh,
					   const Collider& col) :
	mesh(mesh), collider(col), material(LoadMaterialDefault()),
	position(MatrixTranslate(pos.x, pos.y, pos.z)),
	rotation(MatrixRotate({0.0f, 1.0f, 0.0f}, 0.0f)),
	scale(MatrixScale(1.0f, 1.0f, 1.0f))
{

	UploadMesh(&this->mesh, false);
}
PhysObject::PhysObject(const Vector3 pos, const Mesh mesh, const Collider& col,
					   const Shader& shader) : phys::PhysObject(pos, mesh, col)
{
	this->SetShader(shader);
}
PhysObject::PhysObject(const Vector3 pos, const Mesh mesh, const Collider& col,
					   const char* vertShader, const char* fragShader) :
	phys::PhysObject(pos, mesh, col)
{
	this->shader = LoadShader(vertShader, fragShader);
	this->material.shader = this->shader;
}

void PhysObject::Update()
{
	// TODO: Implement update logic
}
void PhysObject::Draw() const
{
	DrawMesh(this->mesh, this->material, this->GetTransformM());
}

auto CreateBoxObject(const Vector3 pos, const Vector3 dims) -> PhysObject
{
	Collider col = CreateBoxCollider(MatrixScale(dims.x, dims.y, dims.z));
	Mesh mesh = GenMeshCube(dims.x, dims.y, dims.z);
#if defined(PLATFORM_WEB)
	static const Shader shader
		= LoadShader(RESOURCES_PATH "shaders/litShader_web.vert",
					 RESOURCES_PATH "shaders/litShader_web.frag");
#else
	static const Shader shader
		= LoadShader(RESOURCES_PATH "shaders/litShader.vert",
					 RESOURCES_PATH "shaders/litShader.frag");
#endif

	return {pos, mesh, col, shader};
}

#ifndef NDEBUG
auto operator<<(ostream& ostr, HitObj& hit) -> ostream&
{
	ostr << hit.HitPos << '\n';
	return ostr;
}
#endif // !NDEBUG

} //namespace phys
