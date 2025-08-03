#include "world.h"

#include <raylib.h>
#include <raymath.h>

void Update();

Camera cam;
phys::World* world;

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

	world = new phys::World();

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
	world->Update();
}
