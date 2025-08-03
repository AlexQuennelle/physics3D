#include "collider.h"
#include "physObject.h"

#include <raylib.h>
#include <raymath.h>

void Update();

Camera cam;

int main()
{
	SetConfigFlags(FLAG_MSAA_4X_HINT);
#if defined(PLATFORM_WEB)
	InitWindow(400, 400, NAME);
	emscripten_set_main_loop(Update, 0, 1);
#else
	InitWindow(800, 800, NAME);
	SetTargetFPS(60);
	{
		using namespace std::numbers;
		cam = *new Camera({.position = Vector3RotateByAxisAngle(
							   Vector3RotateByAxisAngle({5.0f, 0.0f, 0.0f},
														{0.0f, 0.0f, 1.0f},
														pi_v<float> / 12.0f),
							   {0.0f, 1.0f, 0.0f}, pi_v<float> / 4.0f),
						   .target = {0.0f, 0.0f, 0.0f},
						   .up = {0.0f, 1.0f, 0.0f},
						   .fovy = 45.0f,
						   .projection = 0});
	}
	//phys::MeshCollider box = phys::CreateBoxCollider(MatrixIdentity());
	//auto mat = MatrixRotateX(0.0f) * MatrixTranslate(0.0f, 1.2f, 0.0f);
	//phys::CheckCollision(&box, MatrixIdentity(), &box, mat);
	auto box1 = phys::CreateBoxObject({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});
	auto box2 = phys::CreateBoxObject({0.0f, 1.2f, 0.0f}, {1.0f, 1.0f, 1.0f});
	phys::CheckCollision(box1.collider, box1.GetTransformM(), box2.collider,
						 box2.GetTransformM());
	while (!WindowShouldClose())
	{
		float dt = GetFrameTime();
		if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
		{
			Vector2 mouseDelta{GetMouseDelta()};

			cam.position =
				Vector3RotateByAxisAngle(cam.position, {0.0f, 1.0f, 0.0f},
										 -mouseDelta.x * 0.8f * dt);
			float dAngle = -mouseDelta.y * 0.4f * dt;
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

		BeginDrawing();
		ClearBackground({100, 149, 237, 255});
		BeginMode3D(cam);
		DrawMesh(box1.mesh, LoadMaterialDefault(), box1.GetTransformM());
		DrawMesh(box2.mesh, LoadMaterialDefault(), box2.GetTransformM());
		EndMode3D();
		EndDrawing();
		//Update();
	}
#endif

	CloseWindow();

	return 0;
}

void Update()
{
	BeginDrawing();
	ClearBackground({100, 149, 237, 255});
	EndDrawing();
}
