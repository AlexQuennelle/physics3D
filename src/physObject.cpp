#include "physObject.h"
#include "collider.h"

#include <cstdint>
#include <cstdlib>
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
			CheckFaceNors(col1, col2);
			CheckFaceNors(col2, col1);
			//#ifndef NDEBUG
			//			for (auto nor : nors)
			//			{
			//				DrawLine3D({0.0f, 0.0f, 0.0f}, nor, WHITE);
			//			}
			//#endif // !NDEBUG
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
			.ThisCol = obj1, .OtherCol = obj2, .HitPos = {0.0f, 0.0f, 0.0f}};
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
void CheckFaceNors(Col_Sptr col1, Col_Sptr col2)
{
	vector<Vector3> nors;
	col1->GetNormals(nors);
	srand(static_cast<int>(nors[0].x + nors[1].y));
	for (auto nor : nors)
	{
		Range proj1 = col1->GetProjection(nor);
		Range proj2 = col2->GetProjection(nor);
#ifndef NDEBUG
		Color color = {
			static_cast<uint8_t>(rand()),
			static_cast<uint8_t>(rand()),
			static_cast<uint8_t>(rand()),
			255,
		};
		Vector3 support = col2->GetSupportPoint({-nor.x, -nor.y, -nor.z});
		DrawLine3D(col1->origin, col1->origin + nor, color);
		DrawSphere(col1->origin + nor, 0.025f, color);
		DrawSphere(support, 0.05f, color);
#endif // !NDEBUG
	}
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

#ifndef NDEBUG
ostream& operator<<(ostream& ostr, HitObj& hit)
{
	ostr << hit.HitPos << '\n';
	return ostr;
}
#endif // !NDEBUG

} //namespace phys
