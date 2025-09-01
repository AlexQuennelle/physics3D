#include "world.h"
#include "collider.h"
#include "physObject.h"
#include "utils.h"

#include <cassert>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <imgui.h>
#include <iostream>
#include <memory>
#include <numbers>
#include <optional>
#include <raylib.h>
#include <raymath.h>
#include <rlImGui.h>

namespace phys
{

World::World() : imguiIO(ImGui::GetIO())
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
		CreateBoxObject({2.0f, 0.2f, -0.5f}, {1.0f, 1.0f, 1.0f}));
	//CreateBoxObject({2.0f, 0.0f, 0.5f}, {1.0f, 1.0f, 1.0f}));
	//this->objects[0].Rotate(QuaternionFromEuler(0.0f, 45.0f * DEG2RAD, 0.0f));
	//this->objects.push_back(
	//	CreateBoxObject({2.0f, 0.0f, 0.5f}, {1.0f, 1.0f, 1.0f}));
	this->DebugAddStairObj({2.0f, 0.0f, 0.5f});
	//this->objects[1].Rotate(
	//	QuaternionFromAxisAngle({0.0f, 1.0f, 0.0f}, 45.0f * DEG2RAD));
	//this->objects[1].Rotate(
	//	QuaternionFromAxisAngle({1.0f, 0.0f, 0.0f}, -35.0f * DEG2RAD));
	SetTextColor(INFO);
	std::cout << "Done Initializing\n";
	ClearStyles();

	imguiIO.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
}

void World::Update()
{
	this->deltaTime = GetFrameTime();
	BeginDrawing();
	rlImGuiBegin();

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
		for (uint32_t j{i + 1}; j < this->objects.size(); j++)
		{
			if (i == j)
				continue;
			auto obj2 = this->objects[j];
			std::optional<HitObj> col = CheckCollision(obj1, obj2);
			if (col.has_value())
			{
			}
			else
			{
			}
		}
	}
	this->ProcessInput();

	// Drawing logic
	//BeginDrawing();
	//ClearBackground({100, 149, 237, 255});
	//BeginMode3D(cam);
	DrawGrid(2.5, 2);
	for (const auto& obj : this->objects)
	{
		obj.Draw();
	}
	EndMode3D();

	bool open = true;
	//open = selectedObj != nullptr;
	ImGuiWindowFlags flags =
		ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize;
	if (selectedObj != nullptr)
	{
		if (ImGui::Begin("Selected Object", &open, flags))
		{
			// TODO: Add more info
			Vector3 pos;
			Vector3 rot;
			float scale;
			pos = selectedObj->GetPosition();
			rot = QuaternionToEuler(selectedObj->GetRotation()) * RAD2DEG;
			scale = selectedObj->GetScale().x;
			ImGui::DragFloat3("Position", &pos.x, 0.01f);
			ImGui::DragFloat3("Rotation", &rot.x, 0.1f);
			ImGui::DragFloat("Scale", &scale, 0.01f);
			rot = rot * DEG2RAD;
			selectedObj->SetPosition(pos);
			selectedObj->SetRotation(QuaternionFromEuler(rot.x, rot.y, rot.z));
			selectedObj->SetScale(scale);
		}
		ImGui::End();
	}

	rlImGuiEnd();
	EndDrawing();
}
void World::ProcessInput()
{
	if (!imguiIO.WantCaptureMouse)
	{
		if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
		{
			Vector2 mouseDelta{GetMouseDelta()};
			if (!IsCursorHidden())
			{
				DisableCursor();
			}

			cam.position =
				Vector3RotateByAxisAngle(cam.position, {0.0f, 1.0f, 0.0f},
										 -mouseDelta.x * 0.8f * deltaTime);
			float dAngle = -mouseDelta.y * 0.4f * deltaTime;
			float currentAngle = Vector3Angle(Vector3Normalize(cam.position),
											  {0.0f, 1.0f, 0.0f});
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
		else if (IsCursorHidden())
		{
			EnableCursor();
		}
		if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
		{
			selectedObj = nullptr;
			float dist = std::numeric_limits<float>::max();
			for (auto& obj : this->objects)
			{
				auto hit = CheckRaycast(
					GetScreenToWorldRay(GetMousePosition(), this->cam), obj);
				if (hit.has_value())
				{
					DrawSphere(hit->hitPos, 0.025f, GREEN);
					if (hit->hitDist < dist)
					{
						dist = hit->hitDist;
						selectedObj = &hit->hitObj;
					}
				}
			}
		}
		Vector3 camMove = cam.target - cam.position;
		camMove = Vector3Scale(camMove, GetMouseWheelMove() * 0.1f);
		cam.position = Vector3Transform(
			cam.position, MatrixTranslate(camMove.x, camMove.y, camMove.z));
	}
	else
	{
		//if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
		//{
		//	if (!IsCursorHidden())
		//	{
		//		DisableCursor();
		//	}
		//}
		//else if (IsCursorHidden())
		//{
		//	EnableCursor();
		//}
	}
}

// HACK: The following is a hack for testing purposes.
void World::DebugAddStairObj(Vector3 pos)
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

	auto col = std::make_shared<CompoundCollider>(CompoundCollider({
		std::dynamic_pointer_cast<Collider>(
			CreateBoxCollider(MatrixScale(1.0f, 1.0f, 0.5f) *
							  MatrixTranslate(0.0f, 0.0f, 0.25f))),
		std::dynamic_pointer_cast<Collider>(
			CreateBoxCollider(MatrixScale(1.0f, 0.5f, 0.5f) *
							  MatrixTranslate(0.0f, -0.25f, -0.25f))),
	}));
#if defined(PLATFORM_WEB)
	this->objects.emplace_back(PhysObject(
		{0.0f, 0.0f, 0.5f}, mesh, std::dynamic_pointer_cast<Collider>(col),
		RESOURCES_PATH "shaders/litShader_web.vert",
		RESOURCES_PATH "shaders/litShader_web.frag"));
#else
	this->objects.emplace_back(pos, mesh,
							   std::dynamic_pointer_cast<Collider>(col),
							   RESOURCES_PATH "shaders/litShader.vert",
							   RESOURCES_PATH "shaders/litShader.frag");
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
