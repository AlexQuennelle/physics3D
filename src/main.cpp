#include "program.h"

#include <memory>
#include <raylib.h>
#include <raymath.h>
#include <rlImGui.h>
#if defined(PLATFORM_WEB)
#include "wasmUtils.h"
#include <emscripten/emscripten.h>
#endif

void Update();

std::unique_ptr<phys::Program> program{nullptr}; // NOLINT

auto main() -> int
{
	SetConfigFlags(FLAG_MSAA_4X_HINT);
#if defined(PLATFORM_WEB)
	InitWindow(500, 500, NAME);
	rlImGuiSetup(true);
	world = std::make_unique<phys::World>();
	requestSize(500, 500);
	emscripten_set_main_loop(Update, 0, 1);
#else
	InitWindow(800, 800, NAME);
	SetTargetFPS(60);
	rlImGuiSetup(true);
	program = std::make_unique<phys::Program>();
#endif

#if !defined(PLATFORM_WEB)
	while (!WindowShouldClose())
	{
		Update();
	}
#endif

	program.reset(nullptr);

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
	program->Update();
}
