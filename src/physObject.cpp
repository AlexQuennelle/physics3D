#include "physObject.h"
#include "collider.h"
#include "halfEdge.h"

#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <optional>
#include <ranges>
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
#include <set>
#include <utility>
#include <variant>

namespace phys
{

using std::optional;

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

			if (edges.penetration
				< faces1.penetration
				&& edges.penetration
				< faces2.penetration)
			{
				// Edge collision
				std::cout << "Edge Collision\n";
				DrawLine3D(edges.support,
						   edges.support + (edges.normal * edges.penetration),
						   GREEN);
				DrawSphere(edges.support + (edges.normal * edges.penetration),
						   0.01f, GREEN);
			}
			else
			{
				std::cout << "Face Collision\n";
				// Face collision
				const auto& hull1{std::get<0>(col1)};
				const auto& hull2{std::get<0>(col2)};
				if (faces1.penetration < faces2.penetration)
				{
					const auto& ref = hull1.GetFace(faces1.id);
					float dot{1.0f};
					uint8_t incidentID{0};
					for (int i{0}; i < hull2.FaceCount(); i++)
					{
						const auto& face{hull2.GetFace(i)};
						if (float newDot{
								Vector3DotProduct(face.normal, ref.normal)};
							newDot < dot)
						{
							dot = newDot;
							incidentID = i;
						}
					}
					GenFaceContact(ref, hull2.GetFace(incidentID));
					DrawLine3D(faces1.suppport,
							   faces1.suppport
								   + (hull1.GetFace(faces1.id).normal
									  * faces1.penetration),
							   RED);
				}
				else
				{
					DrawLine3D(faces2.suppport,
							   faces2.suppport
								   + (hull2.GetFace(faces2.id).normal
									  * faces2.penetration),
							   RED);
				}
			}
			collision |= true;
		}
	}
#ifndef NDEBUG
	//Draw both objects' colliders in the local coordinate space of object 1
	std::visit([](const isCollider auto& col) -> void
	{
		col.DebugDraw(MatrixIdentity(), {255, 255, 255, 255});
	}, obj1.GetCollider());
	std::visit([&obj1, &obj2](const isCollider auto& col) -> auto
	{
		col.DebugDraw(obj2.GetTransformM() * MatrixInvert(obj1.GetTransformM()),
					  {255, 255, 255, 255});
	}, obj2.GetCollider());
#endif // !NDEBUG
	if (collision)
	{
		// NOTE: Debug visualization code
		std::visit([&obj1](const isCollider auto& col) -> auto
		{
			col.DebugDraw(obj1.GetTransformM(), {255, 0, 0, 255});
		}, obj1.GetCollider());
		std::visit([&obj2](const isCollider auto& col) -> auto
		{
			col.DebugDraw(obj2.GetTransformM(), {255, 0, 0, 255});
		}, obj2.GetCollider());
		HitObj hitObj{
			.HitPos = {0.0f, 0.0f, 0.0f}, .ThisCol = &obj1, .OtherCol = &obj2};
		return hitObj;
	}
	else
	{
		// NOTE: Debug visualization code
		std::visit([&obj1](const isCollider auto& col) -> auto
		{
			col.DebugDraw(obj1.GetTransformM(), {0, 255, 0, 255});
		}, obj1.GetCollider());
		std::visit([&obj2](const isCollider auto& col) -> auto
		{
			col.DebugDraw(obj2.GetTransformM(), {0, 255, 0, 255});
		}, obj2.GetCollider());
	}
	return {};
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
		Vector3 planeNor{
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
				edgeDir = Vector3Negate(edgeDir);

				bothInside = sideTest(sEdge.Next()->Vertex()->Vec())
							 && !sideTest(sEdge.Vertex()->Vec());
			}
			float dist{
				Vector3DotProduct(edgeRef.Vertex()->Vec() - edgeVert, planeNor)
				/ Vector3DotProduct(planeNor, edgeDir)};
			if (dist >= sEdge.Length())
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
		for (auto point : newVerts)
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
		auto end = (Vector3RotateByAxisAngle(Vector3Negate(edge.Dir() * 0.1f),
											 Vector3Negate(ref.normal),
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
			   + Vector3Negate(nor)
			   * Vector3DotProduct(nor, vert.Vec() - refPoint);
	};
	namespace rv = std::views;
	vector<Vector3> contact = sVerts
							  | rv::filter(filter)
							  | rv::transform(transform)
							  | std::ranges::to<vector>();
#ifndef NDEBUG
	for (const auto p : contact)
	{
		DrawSphere(p, 0.01f, RED);
	}
#endif // !NDEBUG
	return contact;
}
auto CheckRaycast(const Ray ray, PhysObject& obj) -> std::optional<RaycastHit>
{
	vector<Collider> colliders;
	std::visit([&obj, &colliders](const isCollider auto& col) -> auto
	{
		col.GetTransformed(obj.GetTransformM(), colliders);
	}, obj.GetCollider());
	RaycastHit hitObj{.hitDist = std::numeric_limits<float>::max(),
					  .hitPos = Vector3Zero(),
					  .hitObj = &obj};
	bool isHit{false};
	for (const auto& collider : colliders)
	{
		const HullCollider& hull{std::get<0>(collider)};
		for (int i{0}; i < hull.FaceCount(); i++)
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
	auto* edge = poly.Edge();
	Vector3 xAxis = edge->Dir();
	Vector3 yAxis = (Vector3CrossProduct(xAxis, poly.normal));
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
	for (int i{0}; i < hull1.faces.size(); i++)
	{
		Vector3 nor = hull1.faces[i].normal;
		Vector3 support{std::visit([nor](const isCollider auto& col) -> Vector3
		{
			return col.GetSupportPoint(Vector3Negate(nor));
		}, colB)};
		float penetration
			= Vector3DotProduct(hull1.faces[i].Edge()->Vertex()->Vec(), nor)
			  - Vector3DotProduct(support, nor);
		if (penetration < hit.penetration)
		{
			hit.penetration = penetration;
			hit.id = i;
			hit.suppport = support;
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
#ifndef NDEBUG
	vector<Vector3> nors;
	std::visit([&nors](const isCollider auto& col) -> auto
	{
		col.GetNormals(nors);
	}, colA);
	srand(static_cast<int>(nors[0].x + nors[1].y));
#endif // !NDEBUG
	EdgeHit hit{};
	hit.penetration = std::numeric_limits<float>::max();

	const auto& hull1 = std::get<0>(colA);
	const auto& hull2 = std::get<0>(colB);

	std::set<int> edges1;
	std::set<int> edges2;

	for (int i{0}; i < hull1.edges.size(); i++)
	{
		if (edges1.contains(i))
		{
			continue;
		}
		auto edge1 = hull1.edges[i];
		edges1.insert(i);
		edges1.insert(hull1.edges[i].twinID);
		for (int j{0}; j < hull2.edges.size(); j++)
		{
			if (edges2.contains(j))
				continue;

			auto edge2 = hull2.edges[j];
			edges2.insert(i);

			Vector3 nor = Vector3Normalize(
				Vector3CrossProduct(edge1.Dir(), edge2.Dir()));
			Vector3 pen
				= (edge1.Vertex()->Vec() + edge1.Next()->Vertex()->Vec())
				  / 2.0f;

			if (Vector3DotProduct(nor, Vector3Normalize(pen - hull1.origin))
				<= 0)
			{
				nor = Vector3Negate(nor);
			}

			Vector3 support{
				std::visit([nor](const isCollider auto& col) -> Vector3
			{
				return col.GetSupportPoint(Vector3Negate(nor));
			}, colB)};
			float penetration
				= std::visit([nor, support](const isCollider auto& col) -> float
			{
				return col.GetProjection(nor).max
					   - Vector3DotProduct(support, nor);
			}, colA);

			if (penetration < hit.penetration)
			{
				hit.penetration = penetration;
				hit.id1 = i;
				hit.id2 = j;
				hit.support = support;
				hit.normal = nor;
			}
#ifndef NDEBUG
			//Color color = {
			//	static_cast<uint8_t>(rand()),
			//	static_cast<uint8_t>(rand()),
			//	static_cast<uint8_t>(rand()),
			//	255,
			//};
			//DrawLine3D(p, p + nor, color);
			//DrawSphere(p + nor, 0.025f, color);
			//DrawSphere(support, 0.05f, color);
#endif // !NDEBUG
		}
	}
	return hit;
}

PhysObject::PhysObject(const Vector3 pos, const Mesh mesh, Collider& col) :
	collider(col), mesh(mesh), position(MatrixTranslate(pos.x, pos.y, pos.z)),
	rotation(MatrixRotate({0.0f, 1.0f, 0.0f}, 0.0f)),
	scale(MatrixScale(1.0f, 1.0f, 1.0f)), material(LoadMaterialDefault())
{

	UploadMesh(&this->mesh, false);
}
PhysObject::PhysObject(const Vector3 pos, const Mesh mesh, Collider col,
					   const Shader& shader) :
	collider(std::move(col)), mesh(mesh)
{
	this->position = MatrixTranslate(pos.x, pos.y, pos.z);
	this->rotation = MatrixRotate({0.0f, 1.0f, 0.0f}, 0.0f);
	this->scale = MatrixScale(1.0f, 1.0f, 1.0f);

	UploadMesh(&this->mesh, false);

	this->material = LoadMaterialDefault();
	this->SetShader(shader);
}
PhysObject::PhysObject(const Vector3 pos, const Mesh mesh, Collider col,
					   const char* vertShader, const char* fragShader) :
	collider(std::move(col)), mesh(mesh)
{
	this->position = MatrixTranslate(pos.x, pos.y, pos.z);
	this->rotation = MatrixRotate({0.0f, 1.0f, 0.0f}, 0.0f);
	this->scale = MatrixScale(1.0f, 1.0f, 1.0f);

	UploadMesh(&this->mesh, false);

	this->material = LoadMaterialDefault();
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

void DisplayObjectInfo(PhysObject& obj) { }

#ifndef NDEBUG
auto operator<<(ostream& ostr, HitObj& hit) -> ostream&
{
	ostr << hit.HitPos << '\n';
	return ostr;
}
#endif // !NDEBUG

} //namespace phys
