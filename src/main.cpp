#include "collider.h"

#include <raylib.h>
#include <raymath.h>

void Update();

int main()
{
	SetConfigFlags(FLAG_MSAA_4X_HINT);
#if defined(PLATFORM_WEB)
	InitWindow(400, 400, NAME);
	emscripten_set_main_loop(Update, 0, 1);
#else
	InitWindow(800, 800, NAME);
	SetTargetFPS(60);
	phys::MeshCollider box = phys::CreateBoxCollider(MatrixIdentity());
	auto mat = MatrixRotateX(0.0f) * MatrixTranslate(0.0f, 1.2f, 0.0f);
	phys::CheckCollision(&box, MatrixIdentity(), &box, mat);
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
	BeginDrawing();
	ClearBackground({100, 149, 237, 255});
	EndDrawing();
}
