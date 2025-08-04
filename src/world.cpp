#include "world.h"
#include "collider.h"
#include "physObject.h"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <raylib.h>
#include <raymath.h>

namespace phys
{
World::World()
{
	using namespace std::numbers;
	cam = *new Camera(
		{.position = Vector3RotateByAxisAngle(
			 Vector3RotateByAxisAngle({5.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f},
									  pi_v<float> / 12.0f),
			 {0.0f, 1.0f, 0.0f}, pi_v<float> * -3.0f / 4.0f),
		 .target = {0.0f, 0.0f, 0.0f},
		 .up = {0.0f, 1.0f, 0.0f},
		 .fovy = 45.0f,
		 .projection = 0});

	this->objects.push_back(
		CreateBoxObject({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f}));
	//this->objects.push_back(
	//	CreateBoxObject({0.0f, 0.0f, 1.1f}, {1.0f, 1.0f, 1.0f}));
	// HACK: The following is a hack for testing purposes.
	Model model = LoadModel(RESOURCES_PATH "stairs.obj");
	//UnloadModel(model);

	Mesh modelMesh = model.meshes[0];
	Mesh mesh;
	mesh.vertexCount = modelMesh.vertexCount;
	mesh.vertices = reinterpret_cast<float*>(
		std::malloc(mesh.vertexCount * 3 * sizeof(float)));
	std::memcpy(mesh.vertices, modelMesh.vertices,
				mesh.vertexCount * 3 * sizeof(float));
	mesh.texcoords = reinterpret_cast<float*>(
		std::malloc(mesh.vertexCount * 2 * sizeof(float)));
	std::memcpy(mesh.texcoords, modelMesh.texcoords,
				mesh.vertexCount * 2 * sizeof(float));
	mesh.normals = reinterpret_cast<float*>(
		std::malloc(mesh.vertexCount * 3 * sizeof(float)));
	std::memcpy(mesh.normals, modelMesh.normals,
				mesh.vertexCount * 3 * sizeof(float));
	mesh.triangleCount = modelMesh.triangleCount;
	//mesh.indices = reinterpret_cast<uint16_t*>(
	//	std::malloc(mesh.triangleCount * 3 * sizeof(uint16_t)));
	//std::memcpy(mesh.indices, modelMesh.indices,
	//			mesh.triangleCount * 3 * sizeof(uint16_t));
	UnloadModel(model);

	//MeshCollider* col = CreateBoxCollider(MatrixScale(0.5f, 0.5f, 0.5f));
	auto* col = new CompoundCollider({
		CreateBoxCollider(MatrixScale(1.0f, 1.0f, 0.5f) *
						  MatrixTranslate(0.0f, 0.0f, 0.25f)),
		CreateBoxCollider(MatrixScale(1.0f, 0.5f, 0.5f) *
						  MatrixTranslate(0.0f, -0.25f, -0.25f)),
	});
	this->objects.emplace_back(*new PhysObject({0.0f, 0.0f, 1.1f}, mesh, col));
	//objects[1].Rotate(
	//	QuaternionFromAxisAngle({1.0f, 0.0f, 0.0f}, -pi_v<float> / 4));
	//std::optional<HitObj> colObj = CheckCollision(
	//	this->objects[0].GetCollider(), this->objects[0].GetTransformM(),
	//	this->objects[1].GetCollider(), this->objects[1].GetTransformM());
	//if (colObj.has_value())
	//{
	//	this->objects[0].SetShaderCol({1.0f, 0.0f, 0.0f, 1.0f});
	//	this->objects[1].SetShaderCol({1.0f, 0.0f, 0.0f, 1.0f});
	//}
	//else
	//{
	//	this->objects[0].SetShaderCol({0.0f, 1.0f, 0.0f, 1.0f});
	//	this->objects[1].SetShaderCol({0.0f, 1.0f, 0.0f, 1.0f});
	//}
}

void World::Update()
{
	this->deltaTime = GetFrameTime();
	objects[1].Rotate(
		QuaternionFromAxisAngle({1.0f, 0.0f, 0.0f}, 1.0f * deltaTime));
	for (auto obj : this->objects)
	{
		obj.Update();
	}
	for (uint32_t i{0}; i < this->objects.size(); i++)
	{
		auto obj1 = this->objects[i];
		for (uint32_t j{0}; j < this->objects.size(); j++)
		{
			if (i == j)
				break;
			auto obj2 = this->objects[j];
			std::optional<HitObj> col =
				CheckCollision(obj1.GetCollider(), obj1.GetTransformM(),
							   obj2.GetCollider(), obj2.GetTransformM());
			if (col.has_value())
			{
				obj1.SetShaderCol({1.0f, 0.0f, 0.0f, 1.0f});
				obj2.SetShaderCol({1.0f, 0.0f, 0.0f, 1.0f});
			}
			else
			{
				obj1.SetShaderCol({0.0f, 1.0f, 0.0f, 1.0f});
				obj2.SetShaderCol({0.0f, 1.0f, 0.0f, 1.0f});
			}
		}
	}
	this->UpdateCamera();
	BeginDrawing();
	ClearBackground({100, 149, 237, 255});
	BeginMode3D(cam);
	for (auto obj : this->objects)
	{
		obj.Draw();
	}
	EndMode3D();
	EndDrawing();
}
void World::UpdateCamera()
{
	if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
	{
		Vector2 mouseDelta{GetMouseDelta()};

		cam.position = Vector3RotateByAxisAngle(
			cam.position, {0.0f, 1.0f, 0.0f}, -mouseDelta.x * 0.8f * deltaTime);
		float dAngle = -mouseDelta.y * 0.4f * deltaTime;
		float currentAngle =
			Vector3Angle(Vector3Normalize(cam.position), {0.0f, 1.0f, 0.0f});
		if (currentAngle + dAngle > 175.0f * DEG2RAD)
		{
			dAngle -= (currentAngle + dAngle) - (175.0f * DEG2RAD);
		}
		else if (currentAngle + dAngle < 5.0f * DEG2RAD)
		{
			dAngle += (5.0f * DEG2RAD) - (currentAngle + dAngle);
		}
		cam.position = Vector3RotateByAxisAngle(
			cam.position,
			Vector3CrossProduct(
				Vector3Subtract({0.0f, 0.0f, 0.0f}, cam.position),
				{0.0f, 1.0f, 0.0f}),
			dAngle);
	}
}

} //namespace phys
