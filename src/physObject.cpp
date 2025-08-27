#include "physObject.h"
#include "collider.h"
#include "imgui.h"

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
#include <utility>

namespace phys
{

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
			vector<Vector3> nors{obj1.GetPosition() - obj2.GetPosition()};
			col1->GetNormals(nors);
			col2->GetNormals(nors);
			GetEdgeCrosses(std::dynamic_pointer_cast<HullCollider>(col1),
						   std::dynamic_pointer_cast<HullCollider>(col2), nors);
			auto faces1 = CheckFaceNors(col1, col2);
			if (faces1.penetration <= 0)
				break;
			auto faces2 = CheckFaceNors(col2, col1);
			if (faces2.penetration <= 0)
				break;
			auto hull1 = std::dynamic_pointer_cast<HullCollider>(col1);
			auto hull2 = std::dynamic_pointer_cast<HullCollider>(col2);
			if (faces1.penetration < faces2.penetration)
			{
				DrawLine3D(faces1.suppport,
						   faces1.suppport + (hull1->GetFace(faces1.id).normal *
											  faces1.penetration),
						   RED);
			}
			else
			{
				DrawLine3D(faces2.suppport,
						   faces2.suppport + (hull2->GetFace(faces2.id).normal *
											  faces2.penetration),
						   RED);
			}
			bool hit = true;
			for (const auto nor : nors)
			{
				Range proj1 = col1->GetProjection(nor);
				Range proj2 = col2->GetProjection(nor);
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
				Vector3DotProduct(ray.direction, face.normal);
			if (dist >= 0)
			{
				auto hitPos = ray.position + (ray.direction * dist);
				bool inPoly{false};
				auto* edge = face.Edge();
				Vector3 xAxis = edge->Dir();
				Vector3 yAxis = (Vector3CrossProduct(xAxis, face.normal));
				Vector2 hitPos2D = {Vector3DotProduct(hitPos, xAxis),
									Vector3DotProduct(hitPos, yAxis)};
				do
				{
					edge = edge->Next();
					Vector2 point1 = {
						Vector3DotProduct(edge->Vertex()->Vec(), xAxis),
						Vector3DotProduct(edge->Vertex()->Vec(), yAxis)};
					point1 = point1 - hitPos2D;
					Vector2 point2 = {
						Vector3DotProduct(edge->Next()->Vertex()->Vec(), xAxis),
						Vector3DotProduct(edge->Next()->Vertex()->Vec(),
										  yAxis)};
					point2 = point2 - hitPos2D;
					if (point1.x < 0 && point2.x < 0)
						continue;
					bool edgeCross{false};
					if ((point1.y > 0 && point2.y <= 0) ||
						(point2.y > 0 && point1.y <= 0))
					{
						if (point1.x == point2.x)
							edgeCross = true;
						else
						{
							float slope =
								(point1.y - point2.y) / (point1.x - point2.x);
							edgeCross = -(point1.y / slope) + point1.x >= 0;
						}
					}
					inPoly = inPoly ^ edgeCross;
				}
				while (edge->Vertex() != face.Edge()->Vertex());
				if (inPoly)
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
Collider::FaceHit CheckFaceNors(Col_Sptr col1, Col_Sptr col2)
{
	Collider::FaceHit hit{};
	vector<Vector3> nors;
	col1->GetNormals(nors);
	srand(static_cast<int>(nors[0].x + nors[1].y));
	auto hull1 = std::dynamic_pointer_cast<HullCollider>(col1);
	for (int i{0}; i < hull1->faces.size(); i++)
	{
		auto nor = hull1->faces[i].normal;
		Range proj1 = col1->GetProjection(nor);
		Vector3 support = col2->GetSupportPoint(Vector3Negate(nor));
		float penertration = proj1.max - Vector3DotProduct(support, nor);
		if (penertration > 0 && penertration < (proj1.max - proj1.min))
		{
			if (penertration > hit.penetration)
			{
				hit.penetration = penertration;
				hit.id = i;
				hit.suppport = support;
			}
			//DrawLine3D(support, support + (nor * penertration), RED);
		}
#ifndef NDEBUG
		Color color = {
			static_cast<uint8_t>(rand()),
			static_cast<uint8_t>(rand()),
			static_cast<uint8_t>(rand()),
			255,
		};
		DrawLine3D(col1->origin, col1->origin + nor, color);
		DrawSphere(col1->origin + nor, 0.025f, color);
		DrawSphere(support, 0.05f, color);
#endif // !NDEBUG
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

void DisplayObjectInfo(PhysObject& obj)
{
}

#ifndef NDEBUG
ostream& operator<<(ostream& ostr, HitObj& hit)
{
	ostr << hit.HitPos << '\n';
	return ostr;
}
#endif // !NDEBUG

} //namespace phys
