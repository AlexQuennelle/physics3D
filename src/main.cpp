#include "collider.h"
#include "physObject.h"

#include <raylib.h>
#include <raymath.h>

void Update();

Camera cam;
phys::PhysObject* box1;
phys::PhysObject* box2;

int main()
{
	SetConfigFlags(FLAG_MSAA_4X_HINT);
#if defined(PLATFORM_WEB)
	InitWindow(400, 400, NAME);
	emscripten_set_main_loop(Update, 0, 1);
#else
	InitWindow(800, 800, NAME);
	SetTargetFPS(60);
#endif
	box1 = new phys::PhysObject(
		phys::CreateBoxObject({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f}));
	box2 = new phys::PhysObject(
		phys::CreateBoxObject({0.0f, 0.0f, 1.1f}, {1.0f, 1.0f, 1.0f}));
	{
		using namespace std::numbers;
		cam = *new Camera({.position = Vector3RotateByAxisAngle(
							   Vector3RotateByAxisAngle({5.0f, 0.0f, 0.0f},
														{0.0f, 0.0f, 1.0f},
														pi_v<float> / 12.0f),
							   {0.0f, 1.0f, 0.0f}, pi_v<float> * -3.0f / 4.0f),
						   .target = {0.0f, 0.0f, 0.0f},
						   .up = {0.0f, 1.0f, 0.0f},
						   .fovy = 45.0f,
						   .projection = 0});
	}
	phys::CheckCollision(box1->collider, box1->GetTransformM(), box2->collider,
						 box2->GetTransformM());
#if !defined(PLATFORM_WEB)
	while (!WindowShouldClose())
	{
		Update();
	}
#endif

	CloseWindow();

	return 0;
}

void Update()
{
	float deltaTime = GetFrameTime();
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

	BeginDrawing();
	ClearBackground({100, 149, 237, 255});
	BeginMode3D(cam);
	DrawMesh(box1->mesh, LoadMaterialDefault(), box1->GetTransformM());
	DrawMesh(box2->mesh, LoadMaterialDefault(), box2->GetTransformM());
	EndMode3D();
	EndDrawing();
}
