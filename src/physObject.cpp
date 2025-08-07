#include "physObject.h"
#include "collider.h"

#include <cstdint>
#include <cstring>
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

namespace phys
{

PhysObject::PhysObject(const Vector3 pos, const Mesh mesh, Collider* col)
{
	this->position = MatrixTranslate(pos.x, pos.y, pos.z);
	this->rotation = MatrixRotate({0.0f, 1.0f, 0.0f}, 0.0f);
	this->scale = MatrixScale(1.0f, 1.0f, 1.0f);
	this->mesh = mesh;
	UploadMesh(&this->mesh, false);
	this->collider = col;
	this->material = LoadMaterialDefault();
}
PhysObject::PhysObject(const Vector3 pos, const Mesh mesh, Collider* col,
					   const Shader& shader)
{
	this->position = MatrixTranslate(pos.x, pos.y, pos.z);
	this->rotation = MatrixRotate({0.0f, 1.0f, 0.0f}, 0.0f);
	this->scale = MatrixScale(1.0f, 1.0f, 1.0f);
	this->mesh = mesh;
	UploadMesh(&this->mesh, false);
	this->collider = col;
	this->material = LoadMaterialDefault();
	this->SetShader(shader);
	this->ColLoc = GetShaderLocation(this->shader, "col");
	this->SetShaderCol({0.0f, 1.0f, 0.0f, 1.0f});
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
	MeshCollider* col = CreateBoxCollider(MatrixScale(dims.x, dims.y, dims.z));
	Mesh mesh;
#if defined(PLATFORM_WEB)
	static const Shader shader =
		LoadShader(RESOURCES_PATH "shaders/litShader_web.vert",
				   RESOURCES_PATH "shaders/litShader_web.frag");
#else
	static const Shader shader =
		LoadShader(RESOURCES_PATH "shaders/litShader.vert",
				   RESOURCES_PATH "shaders/litShader.frag");
#endif

	const vector<Vector3> verts{
		{.x = -dims.x / 2, .y = -dims.y / 2, .z = dims.z / 2},
		{.x = dims.x / 2, .y = -dims.y / 2, .z = dims.z / 2},
		{.x = dims.x / 2, .y = dims.y / 2, .z = dims.z / 2},
		{.x = -dims.x / 2, .y = dims.y / 2, .z = dims.z / 2},
		{.x = -dims.x / 2, .y = -dims.y / 2, .z = -dims.z / 2},
		{.x = -dims.x / 2, .y = dims.y / 2, .z = -dims.z / 2},
		{.x = dims.x / 2, .y = dims.y / 2, .z = -dims.z / 2},
		{.x = dims.x / 2, .y = -dims.y / 2, .z = -dims.z / 2},
		{.x = -dims.x / 2, .y = dims.y / 2, .z = -dims.z / 2},
		{.x = -dims.x / 2, .y = dims.y / 2, .z = dims.z / 2},
		{.x = dims.x / 2, .y = dims.y / 2, .z = dims.z / 2},
		{.x = dims.x / 2, .y = dims.y / 2, .z = -dims.z / 2},
		{.x = -dims.x / 2, .y = -dims.y / 2, .z = -dims.z / 2},
		{.x = dims.x / 2, .y = -dims.y / 2, .z = -dims.z / 2},
		{.x = dims.x / 2, .y = -dims.y / 2, .z = dims.z / 2},
		{.x = -dims.x / 2, .y = -dims.y / 2, .z = dims.z / 2},
		{.x = dims.x / 2, .y = -dims.y / 2, .z = -dims.z / 2},
		{.x = dims.x / 2, .y = dims.y / 2, .z = -dims.z / 2},
		{.x = dims.x / 2, .y = dims.y / 2, .z = dims.z / 2},
		{.x = dims.x / 2, .y = -dims.y / 2, .z = dims.z / 2},
		{.x = -dims.x / 2, .y = -dims.y / 2, .z = -dims.z / 2},
		{.x = -dims.x / 2, .y = -dims.y / 2, .z = dims.z / 2},
		{.x = -dims.x / 2, .y = dims.y / 2, .z = dims.z / 2},
		{.x = -dims.x / 2, .y = dims.y / 2, .z = -dims.z / 2},
	};
	static const vector<float> UVs{
		0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 1.0f,
		0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f,
		1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f,
		0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f,
	};
	const vector<Vector3> normals{
		{.x = 0.0f, .y = 0.0f, .z = 1.0f},	{.x = 0.0f, .y = 0.0f, .z = 1.0f},
		{.x = 0.0f, .y = 0.0f, .z = 1.0f},	{.x = 0.0f, .y = 0.0f, .z = 1.0f},
		{.x = 0.0f, .y = 0.0f, .z = -1.0f}, {.x = 0.0f, .y = 0.0f, .z = -1.0f},
		{.x = 0.0f, .y = 0.0f, .z = -1.0f}, {.x = 0.0f, .y = 0.0f, .z = -1.0f},
		{.x = 0.0f, .y = 1.0f, .z = 0.0f},	{.x = 0.0f, .y = 1.0f, .z = 0.0f},
		{.x = 0.0f, .y = 1.0f, .z = 0.0f},	{.x = 0.0f, .y = 1.0f, .z = 0.0f},
		{.x = 0.0f, .y = -1.0f, .z = 0.0f}, {.x = 0.0f, .y = -1.0f, .z = 0.0f},
		{.x = 0.0f, .y = -1.0f, .z = 0.0f}, {.x = 0.0f, .y = -1.0f, .z = 0.0f},
		{.x = 1.0f, .y = 0.0f, .z = 0.0f},	{.x = 1.0f, .y = 0.0f, .z = 0.0f},
		{.x = 1.0f, .y = 0.0f, .z = 0.0f},	{.x = 1.0f, .y = 0.0f, .z = 0.0f},
		{.x = -1.0f, .y = 0.0f, .z = 0.0f}, {.x = -1.0f, .y = 0.0f, .z = 0.0f},
		{.x = -1.0f, .y = 0.0f, .z = 0.0f}, {.x = -1.0f, .y = 0.0f, .z = 0.0f},
	};

	mesh.vertices =
		reinterpret_cast<float*>(std::malloc(24 * 3 * sizeof(float)));
	std::memcpy(mesh.vertices, verts.data(), 24 * 3 * sizeof(float));
	mesh.texcoords =
		reinterpret_cast<float*>(std::malloc(24 * 2 * sizeof(float)));
	std::memcpy(mesh.texcoords, UVs.data(), 24 * 2 * sizeof(float));
	mesh.normals =
		reinterpret_cast<float*>(std::malloc(24 * 3 * sizeof(float)));
	std::memcpy(mesh.normals, normals.data(), 24 * 3 * sizeof(float));

	mesh.indices =
		reinterpret_cast<uint16_t*>(std::malloc(36 * sizeof(uint16_t)));

	int k__ = 0;
	for (int i = 0; i < 36; i += 6)
	{
		mesh.indices[i] = 4 * k__;
		mesh.indices[i + 1] = 4 * k__ + 1;
		mesh.indices[i + 2] = 4 * k__ + 2;
		mesh.indices[i + 3] = 4 * k__;
		mesh.indices[i + 4] = 4 * k__ + 2;
		mesh.indices[i + 5] = 4 * k__ + 3;

		k__++;
	}

	mesh.vertexCount = 24;
	mesh.triangleCount = 12;

	return {pos, mesh, col, shader};
}

} //namespace phys
