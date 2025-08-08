#include "world.h"
#include "collider.h"
#include "physObject.h"
#include "utils.h"

#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <numbers>
#include <raylib.h>
#include <raymath.h>

namespace phys
{
World::World()
{
	SetTextColor(INFO);
	std::cout << "Initializing World\n";
	ClearStyles();
	using namespace std::numbers;
	this->cam = Camera(
		{.position = Vector3RotateByAxisAngle(
			 Vector3RotateByAxisAngle({10.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f},
									  pi_v<float> / 12.0f),
			 {0.0f, 1.0f, 0.0f}, pi_v<float> * -3.0f / 4.0f),
		 .target = {0.0f, 0.0f, 0.0f},
		 .up = {0.0f, 1.0f, 0.0f},
		 .fovy = 45.0f,
		 .projection = 0});

	this->objects.push_back(
		CreateBoxObject({0.0f, 0.0f, -0.75f}, {1.0f, 1.0f, 1.0f}));
	this->objects[0].Rotate(QuaternionFromEuler(0.0f, 45.0f * DEG2RAD, 0.0f));
	//this->objects.push_back(
	//	CreateBoxObject({0.0f, 0.0f, 1.1f}, {1.0f, 1.0f, 1.0f}));
	this->DebugAddStairObj();
	this->objects[1].Rotate(QuaternionFromEuler(-45.0f * DEG2RAD, 0.0f, 0.0f));
	SetTextColor(INFO);
	std::cout << "Done Initializing\n";
	ClearStyles();
	//auto discard = CheckCollision(
	//	this->objects[0].GetCollider(), this->objects[0].GetTransformM(),
	//	this->objects[1].GetCollider(), this->objects[1].GetTransformM());
}

void World::Update()
{
	this->deltaTime = GetFrameTime();
	BeginDrawing();
	ClearBackground({100, 149, 237, 255});
	BeginMode3D(cam);
	//objects[1].Rotate(
	//	QuaternionFromAxisAngle({1.0f, 0.0f, 0.0f}, 1.0f * deltaTime));
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
			}
			else
			{
			}
		}
	}
	this->UpdateCamera();

	// Drawing logic
	//BeginDrawing();
	//ClearBackground({100, 149, 237, 255});
	//BeginMode3D(cam);
	DrawGrid(2.5, 2);
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
	Vector3 camMove = cam.target - cam.position;
	camMove = Vector3Scale(camMove, GetMouseWheelMove() * 0.1f);
	cam.position = Vector3Transform(
		cam.position, MatrixTranslate(camMove.x, camMove.y, camMove.z));
}

// HACK: The following is a hack for testing purposes.
void World::DebugAddStairObj()
{
	Model model = LoadModel(RESOURCES_PATH "stairs.obj");

	Mesh modelMesh = model.meshes[0];
	Mesh mesh{0};
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
	mesh.indices = nullptr;
	UnloadModel(model);

	auto* col = new CompoundCollider({
		CreateBoxCollider(MatrixScale(1.0f, 1.0f, 0.5f) *
						  MatrixTranslate(0.0f, 0.0f, 0.25f)),
		CreateBoxCollider(MatrixScale(1.0f, 0.5f, 0.5f) *
						  MatrixTranslate(0.0f, -0.25f, -0.25f)),
	});
#if defined(PLATFORM_WEB)
	this->objects.emplace_back(
		PhysObject({0.0f, 0.0f, 1.25f}, mesh, col,
				   RESOURCES_PATH "shaders/litShader_web.vert",
				   RESOURCES_PATH "shaders/litShader_web.frag"));
#else
	this->objects.emplace_back(PhysObject(
		{0.0f, 0.0f, 0.5f}, mesh, col, RESOURCES_PATH "shaders/litShader.vert",
		RESOURCES_PATH "shaders/litShader.frag"));
#endif // defined ()
}

void DrawGrid(const float lineLength, const int count)
{
	for (int i{-count}; i < (count + 1); i++)
	{
		DrawLine3D({static_cast<float>(i), 0.0f, -lineLength},
				   {static_cast<float>(i), 0.0f, lineLength},
				   {100, 100, 100, 255});
		DrawLine3D({-lineLength, 0.0f, static_cast<float>(i)},
				   {lineLength, 0.0f, static_cast<float>(i)},
				   {100, 100, 100, 255});
	}
	DrawLine3D({0.0f, 0.0f, 0.0f}, {-lineLength, 0.0f, 0.0f}, {0, 0, 0, 255});
	DrawLine3D({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -lineLength}, {0, 0, 0, 255});
	DrawLine3D({0.0f, 0.0f, 0.0f}, {lineLength, 0.0f, 0.0f}, {255, 0, 0, 255});
	DrawLine3D({0.0f, 0.0f, 0.0f}, {0.0f, lineLength, 0.0f}, {0, 255, 0, 255});
	DrawLine3D({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, lineLength}, {0, 0, 255, 255});
}

} //namespace phys
