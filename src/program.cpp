#include "program.h"
#include "collider.h"
#include "physObject.h"
#include "utils.h"

#include <cassert>
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

Program::Program() : deltaTime(NAN), cam({}), imguiIO(&ImGui::GetIO())
{
	SetTextColor(INFO);
	std::cout << "Initializing Program\n";
	ClearStyles();
	using namespace std::numbers;
	this->cam = Camera({
		.position = Vector3RotateByAxisAngle(
			Vector3RotateByAxisAngle({.x = 10.0f, .y = 0.0f, .z = 0.0f},
									 {.x = 0.0f, .y = 0.0f, .z = 1.0f},
									 pi_v<float> / 12.0f),
			{.x = 0.0f, .y = 1.0f, .z = 0.0f}, pi_v<float> * -3.0f / 4.0f),
		.target = {.x = 0.0f, .y = 0.0f, .z = 0.0f},
		.up = {.x = 0.0f, .y = 1.0f, .z = 0.0f},
		.fovy = 45.0f,
		.projection = 0,
	});

#ifdef PLATFORM_WEB
	this->objects.push_back(CreateBoxObject({.x = 0.0f, .y = 0.0f, .z = 0.0f},
											{.x = 1.0f, .y = 1.0f, .z = 1.0f}));
	this->DebugAddStairObj({.x = 0.0f, .y = 0.0f, .z = 1.0f});
	this->objects.at(1).Rotate(QuaternionFromAxisAngle(
		{.x = 1.0f, .y = 0.0f, .z = 0.0f}, -45.0f * DEG2RAD));
#else
	this->objects.push_back(CreateBoxObject({.x = 2.0f, .y = 0.2f, .z = -0.5f},
											{.x = 1.0f, .y = 1.0f, .z = 1.0f}));
	this->DebugAddStairObj({.x = 2.0f, .y = 0.0f, .z = 0.5f});
#endif // PLATFORM_WEB
	SetTextColor(INFO);
	std::cout << "Done Initializing\n";
	ClearStyles();

	imguiIO->ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
}

void Program::Update()
{
	this->deltaTime = GetFrameTime();
	BeginDrawing();
	rlImGuiBegin();

	ClearBackground({.r = 100, .g = 149, .b = 237, .a = 255});
	BeginMode3D(cam);
	for (auto& obj : this->objects)
	{
		obj.Update();
	}
	for (uint32_t i{0}; i < this->objects.size(); i++)
	{
		const auto& obj1 = this->objects[i];
		for (uint32_t j{i + 1}; j < this->objects.size(); j++)
		{
			if (i == j)
				continue;
			const auto& obj2 = this->objects[j];
			std::optional<HitObj> col = CheckCollision(obj1, obj2);
			if (col.has_value())
			{
				col.value();
			}
			else
			{
			}
		}
	}
	this->ProcessInput();

	// Drawing logic
	DrawGrid(2.5f, 2);
	for (const auto& obj : this->objects)
	{
		obj.Draw();
	}
	EndMode3D();

	bool open = true;
	ImGuiWindowFlags flags
		= ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize;
	if (selectedObj != nullptr)
	{
		if (ImGui::Begin("Selected Object", &open, flags))
		{
			// TODO: Add more info
			Vector3 pos;
			Vector3 rot;
			float scale = selectedObj->GetScale().x;
			pos = selectedObj->GetPosition();
			rot = QuaternionToEuler(selectedObj->GetRotation()) * RAD2DEG;
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

	DrawFPS(0, 0);

	rlImGuiEnd();
	EndDrawing();
}
void Program::ProcessInput()
{
	if (!imguiIO->WantCaptureMouse)
	{
		if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
		{
			Vector2 mouseDelta{GetMouseDelta()};
			if (!IsCursorHidden())
			{
				DisableCursor();
			}

			cam.position = Vector3RotateByAxisAngle(
				cam.position, {.x = 0.0f, .y = 1.0f, .z = 0.0f},
				-mouseDelta.x * 0.8f * deltaTime);
			float dAngle = -mouseDelta.y * 0.4f * deltaTime;
			float currentAngle
				= Vector3Angle(Vector3Normalize(cam.position),
							   {.x = 0.0f, .y = 1.0f, .z = 0.0f});
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
					Vector3Subtract({.x = 0.0f, .y = 0.0f, .z = 0.0f},
									cam.position),
					{.x = 0.0f, .y = 1.0f, .z = 0.0f}),
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
						selectedObj = hit->hitObj;
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
void Program::DebugAddStairObj(Vector3 pos)
{
	Model model = LoadModel(RESOURCES_PATH "stairs.obj");

	Mesh modelMesh = *model.meshes;
	Mesh mesh{
		.vertexCount = 0,
		.triangleCount = 0,
		.vertices = nullptr,
		.texcoords = nullptr,
		.texcoords2 = nullptr,
		.normals = nullptr,
		.tangents = nullptr,
		.colors = nullptr,
		.indices = nullptr,
		.boneCount = 0,
		// .boneIndices = nullptr,
		.boneWeights = nullptr,
		.animVertices = nullptr,
		.animNormals = nullptr,
		.vaoId = 0,
		.vboId = nullptr,
	};
	mesh.vertexCount = modelMesh.vertexCount;
	mesh.vertices = static_cast<float*>(std::malloc(
		static_cast<uint64_t>(mesh.vertexCount * 3) * sizeof(float)));
	std::memcpy(mesh.vertices, modelMesh.vertices,
				static_cast<uint64_t>(mesh.vertexCount * 3) * sizeof(float));
	mesh.texcoords = static_cast<float*>(std::malloc(
		static_cast<uint64_t>(mesh.vertexCount * 2) * sizeof(float)));
	std::memcpy(mesh.texcoords, modelMesh.texcoords,
				static_cast<uint64_t>(mesh.vertexCount * 2) * sizeof(float));
	mesh.normals = static_cast<float*>(std::malloc(
		static_cast<uint64_t>(mesh.vertexCount * 3) * sizeof(float)));
	std::memcpy(mesh.normals, modelMesh.normals,
				static_cast<uint64_t>(mesh.vertexCount * 3) * sizeof(float));
	mesh.triangleCount = modelMesh.triangleCount;
	mesh.indices = nullptr;
	UnloadModel(model);

	auto col = CompoundCollider({
		CreateBoxCollider(MatrixScale(1.0f, 1.0f, 0.5f)
						  * MatrixTranslate(0.0f, 0.0f, 0.25f)),
		CreateBoxCollider(MatrixScale(1.0f, 0.5f, 0.5f)
						  * MatrixTranslate(0.0f, -0.25f, -0.25f)),
	});
#ifdef PLATFORM_WEB
	this->objects.emplace_back(pos, mesh, col,
							   RESOURCES_PATH "shaders/litShader_web.vert",
							   RESOURCES_PATH "shaders/litShader_web.frag");
#else
	this->objects.emplace_back(pos, mesh, col,
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
