#include "physObject.h"
#include "collider.h"

#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
#include <utility>

namespace phys
{

PhysObject::PhysObject(const Vector3 pos, const Mesh mesh, Col_Uptr col)
{
	this->position = MatrixTranslate(pos.x, pos.y, pos.z);
	this->rotation = MatrixRotate({0.0f, 1.0f, 0.0f}, 0.0f);
	this->scale = MatrixScale(1.0f, 1.0f, 1.0f);
	this->mesh = mesh;
	UploadMesh(&this->mesh, false);
	this->collider = std::move(col);
	this->material = LoadMaterialDefault();
}
PhysObject::PhysObject(const Vector3 pos, const Mesh mesh, Col_Uptr col,
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
PhysObject::PhysObject(const Vector3 pos, const Mesh mesh, Col_Uptr col,
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
	// TODO: Make rotation work
	Col_Uptr col =
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

	return {pos, mesh, std::move(col), shader};
}

} //namespace phys
