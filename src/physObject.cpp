#include "physObject.h"
#include "collider.h"
#include "halfEdge.h"

#include <cassert>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
#include <set>
#include <utility>

namespace phys
{

/**
 * Tests if a 3D point planar to a face lies within the polygon described by
 * its edges.
 */
bool IsPointInPoly3D(const Vector3 point, const HE::HFace& poly);

void GenFaceContact(const HE::HFace& ref, const HE::HFace& incident);

std::optional<HitObj> CheckCollision(const PhysObject& obj1,
									 const PhysObject& obj2)
{
	vector<Col_Sptr> cols1;
	obj1.GetColliderT(MatrixIdentity(), cols1);
	vector<Col_Sptr> cols2;
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

			if (edges.penetration < faces1.penetration &&
				edges.penetration < faces2.penetration)
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
				auto hull1 = std::dynamic_pointer_cast<HullCollider>(col1);
				auto hull2 = std::dynamic_pointer_cast<HullCollider>(col2);
				if (faces1.penetration < faces2.penetration)
				{
					const auto& ref = hull1->GetFace(faces1.id);
					float dot{1.0f};
					uint8_t incidentID;
					for (int i{0}; i < hull2->FaceCount(); i++)
					{
						const auto& face{hull2->GetFace(i)};
						if (float newDot{
								Vector3DotProduct(face.normal, ref.normal)};
							newDot < dot)
						{
							dot = newDot;
							incidentID = i;
						}
					}
					GenFaceContact(ref, hull2->GetFace(incidentID));
					DrawLine3D(faces1.suppport,
							   faces1.suppport +
								   (hull1->GetFace(faces1.id).normal *
									faces1.penetration),
							   RED);
				}
				else
				{
					DrawLine3D(faces2.suppport,
							   faces2.suppport +
								   (hull2->GetFace(faces2.id).normal *
									faces2.penetration),
							   RED);
				}
			}
			collision |= true;
		}
	}
#ifndef NDEBUG
	//Draw both objects' colliders in the local coordinate space of object 1
	obj1.GetCollider()->DebugDraw(MatrixIdentity(), {255, 255, 255, 255});
	obj2.GetCollider()->DebugDraw(obj2.GetTransformM() *
									  MatrixInvert(obj1.GetTransformM()),
								  {255, 255, 255, 255});
#endif // !NDEBUG
	if (collision)
	{
		// NOTE: Debug visualization code
		obj1.GetCollider()->DebugDraw(obj1.GetTransformM(), {255, 0, 0, 255});
		obj2.GetCollider()->DebugDraw(obj2.GetTransformM(), {255, 0, 0, 255});
		HitObj hitObj{
			.HitPos = {0.0f, 0.0f, 0.0f}, .ThisCol = obj1, .OtherCol = obj2};
		return hitObj;
	}
	else
	{
		// NOTE: Debug visualization code
		obj1.GetCollider()->DebugDraw(obj1.GetTransformM(), {0, 255, 0, 255});
		obj2.GetCollider()->DebugDraw(obj2.GetTransformM(), {0, 255, 0, 255});
	}
	return {};
}
void GenFaceContact(const HE::HFace& ref, const HE::HFace& incident)
{
	// Clip algo
	auto plane = GenMeshPlane(0.5f, 0.5f, 1, 1);
	vector<HE::HEdge> surface;
	vector<HE::HVertex> sVerts;
	// Iterate over incident edges to create a surface shape
	HE::HEdge* iEdge{incident.Edge()};
	do
	{
		iEdge = iEdge->Next();
		sVerts.push_back(*iEdge->Vertex());
		HE::HEdge newEdge{
			.vertID = static_cast<uint8_t>(sVerts.size() - 1),
			.twinID = 0,
			.nextID = static_cast<uint8_t>(surface.size() + 1),
			.faceID = 0,
			.vertArr = &sVerts,
			.edgeArr = &surface,
			.faceArr = nullptr,
		};
		std::cout << surface.size() + 1 << '\n';
		surface.push_back(newEdge);
	}
	while (iEdge->Vertex() != incident.Edge()->Vertex());
	surface[surface.size() - 1].nextID = 0; // Close the loop

	// Iterate over reference face's edges to clip the surface
	HE::HEdge* edgeRef{ref.Edge()};
	do
	{
		edgeRef = edgeRef->Next();
		Vector3 planeNor{
			Vector3Normalize(Vector3CrossProduct(ref.normal, edgeRef->Dir()))};
		auto trans = QuaternionToMatrix(
			QuaternionFromVector3ToVector3({0.0f, 1.0f, 0.0f}, planeNor));
		trans =
			trans * MatrixTranslate(edgeRef->Center().x, edgeRef->Center().y,
									edgeRef->Center().z);
#ifndef NDEBUG
		DrawMesh(plane, LoadMaterialDefault(), trans);
		DrawLine3D(edgeRef->Center(), edgeRef->Center() + (planeNor * 0.25f),
				   BLUE);
#endif // !NDEBUG

		// Iterate over surface and clip it against the current reference edge.
		// Clipped surface goes in new vectors, that are then swapped in with
		// the old ones
		vector<HE::HEdge> newSurface{};
		vector<HE::HVertex> newVerts{};
		HE::HEdge* sEdge{surface.data()};
		do
		{
			sEdge = sEdge->Next();
			auto edgeDir{sEdge->Dir()};
			auto edgeVert{sEdge->Next()->Vertex()->Vec()};
			if (Vector3DotProduct(planeNor, edgeDir) < 0)
			{
				edgeVert = sEdge->Vertex()->Vec();
				edgeDir = Vector3Negate(edgeDir);
			}
			float dist{Vector3DotProduct(edgeRef->Vertex()->Vec() - edgeVert,
										 planeNor) /
					   Vector3DotProduct(planeNor, edgeDir)};
			if (dist < 0)
			{
				continue;
			}
			else if (dist > sEdge->Length())
			{
				newVerts.push_back(*sEdge->Vertex());
				HE::HEdge newEdge{
					.vertID = static_cast<uint8_t>(newVerts.size() - 1),
					.twinID = 0,
					.nextID = static_cast<uint8_t>(newSurface.size() + 1),
					.faceID = 0,
					.vertArr = &sVerts,
					.edgeArr = &surface,
					.faceArr = nullptr,
				};
				std::cout << newSurface.size() + 1 << '\n';
				newSurface.push_back(newEdge);
				continue;
			}
			else
			{
				Vector3 newPos{edgeVert + (edgeDir * dist)};
				newPos = newPos +
						 (Vector3Negate(ref.normal) *
						  Vector3DotProduct(newPos - edgeRef->Vertex()->Vec(),
											ref.normal));
				newVerts.emplace_back(newPos.x, newPos.y, newPos.z,
									  newSurface.size());
				HE::HEdge newEdge{
					.vertID = static_cast<uint8_t>(newVerts.size() - 1),
					.twinID = 0,
					.nextID = static_cast<uint8_t>(newSurface.size() + 1),
					.faceID = 0,
					.vertArr = &sVerts,
					.edgeArr = &surface,
					.faceArr = nullptr,
				};
				std::cout << newSurface.size() + 1 << '\n';
				newSurface.push_back(newEdge);
				continue;
			}
			//raise(SIGTRAP);
			break;
		}
		while (sEdge->Vertex() != surface.data()->Vertex());
		std::cout << newSurface.size() << '\n';
		newSurface[newSurface.size() - 1].nextID = 0; // Close the loop
		//sVerts = newVerts;
		//surface = newSurface;
		//for (auto& edge : surface)
		//{
		//	edge.vertArr = &sVerts;
		//	edge.edgeArr = &surface;
		//}

		break;
		HE::HEdge* edge{incident.Edge()};
		do
		{
			edge = edge->Next();
			auto edgeDir{edge->Dir()};
			auto edgeVert{edge->Next()->Vertex()->Vec()};
			if (Vector3DotProduct(planeNor, edgeDir) < 0)
			{
				edgeVert = edge->Vertex()->Vec();
				edgeDir = Vector3Negate(edgeDir);
			}
			float dist{Vector3DotProduct(edgeRef->Vertex()->Vec() - edgeVert,
										 planeNor) /
					   Vector3DotProduct(planeNor, edgeDir)};
			if (dist < 0 || dist > edge->Length())
				continue;
			else
			{
				Vector3 newPos{edgeVert + (edgeDir * dist)};
				newPos = newPos +
						 (Vector3Negate(ref.normal) *
						  Vector3DotProduct(newPos - edgeRef->Vertex()->Vec(),
											ref.normal));
				DrawSphere(newPos, 0.025f, RED);
				DrawSphere(edgeVert, 0.025f, GREEN);
			}
		}
		while (edge->Vertex() != incident.Edge()->Vertex());
	}
	while (edgeRef->Vertex() != ref.Edge()->Vertex());
	UnloadMesh(plane);

	auto* sEdge{surface.data()};
	do
	{
		sEdge = sEdge->Next();
		auto newVec = sEdge->Vertex()->Vec() +
					  (Vector3Negate(ref.normal) *
					   Vector3DotProduct(sEdge->Vertex()->Vec() -
											 edgeRef->Vertex()->Vec(),
										 ref.normal));
		sEdge->Vertex()->SetPos(newVec);
		DrawSphere(sEdge->Vertex()->Vec(), 0.025f, YELLOW);
	}
	while (sEdge->Vertex() != surface[0].Vertex());
}
std::optional<RaycastHit> CheckRaycast(const Ray ray, PhysObject& obj)
{
	vector<Col_Sptr> colliders;
	obj.GetCollider()->GetTransformed(obj.GetTransformM(), colliders);
	RaycastHit hitObj{.hitDist = std::numeric_limits<float>::max(),
					  .hitPos = Vector3Zero(),
					  .hitObj = obj};
	bool isHit{false};
	for (const auto& collider : colliders)
	{
		HullCollider hull = *std::dynamic_pointer_cast<HullCollider>(collider);
		for (int i{0}; i < hull.FaceCount(); i++)
		{
			const auto& face = hull.GetFace(i);
			if (Vector3DotProduct(face.normal, ray.direction) > 0)
				continue;
			float dist =
				Vector3DotProduct(face.Edge()->Vertex()->Vec() - ray.position,
								  face.normal) /
				Vector3DotProduct(face.normal, ray.direction);
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
				continue;
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
bool IsPointInPoly3D(const Vector3 point, const HE::HFace& poly)
{
	bool inPoly{false};
	auto* edge = poly.Edge();
	Vector3 xAxis = edge->Dir();
	Vector3 yAxis = (Vector3CrossProduct(xAxis, poly.normal));
	Vector2 point2D = {Vector3DotProduct(point, xAxis),
					   Vector3DotProduct(point, yAxis)};
	do
	{
		edge = edge->Next();
		Vector2 point1 = {Vector3DotProduct(edge->Vertex()->Vec(), xAxis),
						  Vector3DotProduct(edge->Vertex()->Vec(), yAxis)};
		point1 = point1 - point2D;
		Vector2 point2 = {
			Vector3DotProduct(edge->Next()->Vertex()->Vec(), xAxis),
			Vector3DotProduct(edge->Next()->Vertex()->Vec(), yAxis)};
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
	while (edge->Vertex() != poly.Edge()->Vertex());
	return inPoly;
}
Collider::FaceHit CheckFaceNors(Col_Sptr col1, Col_Sptr col2)
{
#ifndef NDEBUG
	vector<Vector3> nors;
	col1->GetNormals(nors);
	srand(static_cast<int>(nors[0].x + nors[1].y));
#endif // !NDEBUG
	Collider::FaceHit hit{};
	hit.penetration = std::numeric_limits<float>::max();
	auto hull1 = std::dynamic_pointer_cast<HullCollider>(col1);
	for (int i{0}; i < hull1->faces.size(); i++)
	{
		Vector3 nor = hull1->faces[i].normal;
		Vector3 support = col2->GetSupportPoint(Vector3Negate(nor));
		float penetration =
			Vector3DotProduct(hull1->faces[i].Edge()->Vertex()->Vec(), nor) -
			Vector3DotProduct(support, nor);
		if (penetration < hit.penetration)
		{
			hit.penetration = penetration;
			hit.id = i;
			hit.suppport = support;
		}
		//DrawLine3D(support, support + (nor * penertration), RED);
#ifndef NDEBUG
		//Color color = {
		//	static_cast<uint8_t>(rand()),
		//	static_cast<uint8_t>(rand()),
		//	static_cast<uint8_t>(rand()),
		//	255,
		//};
		//DrawLine3D(col1->origin, col1->origin + nor, color);
		//DrawSphere(col1->origin + nor, 0.025f, color);
		//DrawSphere(support, 0.05f, color);
#endif // !NDEBUG
	}
	return hit;
}
Collider::EdgeHit CheckEdgeNors(Col_Sptr col1, Col_Sptr col2)
{
#ifndef NDEBUG
	vector<Vector3> nors;
	col1->GetNormals(nors);
	srand(static_cast<int>(nors[0].x + nors[1].y));
#endif // !NDEBUG
	Collider::EdgeHit hit{};
	hit.penetration = std::numeric_limits<float>::max();
	auto hull1 = std::dynamic_pointer_cast<HullCollider>(col1);
	auto hull2 = std::dynamic_pointer_cast<HullCollider>(col2);
	std::set<int> edges1;
	std::set<int> edges2;
	for (int i{0}; i < hull1->edges.size(); i++)
	{
		if (edges1.contains(i))
		{
			continue;
		}
		auto edge1 = hull1->edges[i];
		edges1.insert(i);
		edges1.insert(hull1->edges[i].twinID);
		for (int j{0}; j < hull2->edges.size(); j++)
		{
			if (edges2.contains(j))
			{
				continue;
			}
			auto edge2 = hull2->edges[j];
			edges2.insert(i);
			//edges2.insert(hull2->edges[j].twinID);
			Vector3 nor =
				Vector3Normalize(Vector3CrossProduct(edge1.Dir(), edge2.Dir()));
			Vector3 p =
				(edge1.Vertex()->Vec() + edge1.Next()->Vertex()->Vec()) / 2;
			if (Vector3DotProduct(nor, Vector3Normalize(p - hull1->origin)) <=
				0)
			{
				nor = Vector3Negate(nor);
			}
			Vector3 support = col2->GetSupportPoint(Vector3Negate(nor));
			float penetration =
				col1->GetProjection(nor).max - Vector3DotProduct(support, nor);
			if (penetration < hit.penetration)
			{
				hit.penetration = penetration;
				hit.id1 = i;
				hit.id2 = j;
				hit.support = support;
				hit.normal = nor;
			}
			//DrawLine3D(p, p + (nor * 0.5f), RED);
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

PhysObject::PhysObject(const Vector3 pos, const Mesh mesh, Col_Sptr col)
{
	this->position = MatrixTranslate(pos.x, pos.y, pos.z);
	this->rotation = MatrixRotate({0.0f, 1.0f, 0.0f}, 0.0f);
	this->scale = MatrixScale(1.0f, 1.0f, 1.0f);
	this->mesh = mesh;
	UploadMesh(&this->mesh, false);
	this->collider = std::move(col);
	this->material = LoadMaterialDefault();
}
PhysObject::PhysObject(const Vector3 pos, const Mesh mesh, Col_Sptr col,
					   const Shader& shader)
{
	this->position = MatrixTranslate(pos.x, pos.y, pos.z);
	this->rotation = MatrixRotate({0.0f, 1.0f, 0.0f}, 0.0f);
	this->scale = MatrixScale(1.0f, 1.0f, 1.0f);
	this->mesh = mesh;
	UploadMesh(&this->mesh, false);
	this->collider = std::move(col);
	this->material = LoadMaterialDefault();
	this->SetShader(shader);
}
PhysObject::PhysObject(const Vector3 pos, const Mesh mesh, Col_Sptr col,
					   const char* vertShader, const char* fragShader)
{
	this->position = MatrixTranslate(pos.x, pos.y, pos.z);
	this->rotation = MatrixRotate({0.0f, 1.0f, 0.0f}, 0.0f);
	this->scale = MatrixScale(1.0f, 1.0f, 1.0f);
	this->mesh = mesh;
	UploadMesh(&this->mesh, false);
	this->collider = std::move(col);
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

PhysObject CreateBoxObject(const Vector3 pos, const Vector3 dims)
{
	std::shared_ptr<HullCollider> col =
		CreateBoxCollider(MatrixScale(dims.x, dims.y, dims.z));
	Mesh mesh = GenMeshCube(dims.x, dims.y, dims.z);
#if defined(PLATFORM_WEB)
	static const Shader shader =
		LoadShader(RESOURCES_PATH "shaders/litShader_web.vert",
				   RESOURCES_PATH "shaders/litShader_web.frag");
#else
	static const Shader shader =
		LoadShader(RESOURCES_PATH "shaders/litShader.vert",
				   RESOURCES_PATH "shaders/litShader.frag");
#endif

	return {pos, mesh, col, shader};
}

void DisplayObjectInfo(PhysObject& obj) {}

#ifndef NDEBUG
ostream& operator<<(ostream& ostr, HitObj& hit)
{
	ostr << hit.HitPos << '\n';
	return ostr;
}
#endif // !NDEBUG

} //namespace phys
