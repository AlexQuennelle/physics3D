#include "world.h"
#include "physObject.h"

#include <raylib.h>

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
	this->objects.push_back(
		CreateBoxObject({0.0f, 0.0f, 1.1f}, {1.0f, 1.0f, 1.0f}));
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
