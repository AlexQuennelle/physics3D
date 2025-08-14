#include "world.h"

#include <raylib.h>
#include <raymath.h>
#include <rlImGui.h>
#if defined(PLATFORM_WEB)
#include <emscripten/emscripten.h>
#endif

void Update();

phys::World* world;

int main()
{
	SetConfigFlags(FLAG_MSAA_4X_HINT);
#if defined(PLATFORM_WEB)
	InitWindow(500, 500, NAME);
	rlImGuiSetup(true);
	world = new phys::World();
	emscripten_set_main_loop(Update, 0, 1);
#else
	InitWindow(800, 800, NAME);
	SetTargetFPS(60);
	rlImGuiSetup(true);
	world = new phys::World();
#endif

#if !defined(PLATFORM_WEB)
	while (!WindowShouldClose())
	{
		Update();
	}
#endif

	rlImGuiShutdown();
	CloseWindow();

	return 0;
}

void Update()
{
	//if (IsKeyPressed(KEY_S))
	//{
	//	TakeScreenshot("screenshot.png");
	//}
	world->Update();
}
