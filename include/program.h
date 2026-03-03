#pragma once

#include "mesh.h"
#include "window.h"
#include "matrix.h"

#include <bx/timer.h>

static const std::string_view Backend{BACKEND};

class Program
{
	public:
	Program();
	Program(const Program&) = delete;
	Program(Program&&) = delete;
	~Program();

	void Run();

	auto operator=(const Program&) -> Program& = delete;
	auto operator=(Program&&) -> Program& = delete;

#ifdef __EMSCRIPTEN__
	friend void WebLoop(void* arg);
#endif // __EMSCRIPTEN__

	private:
	void Init();
	void Update();
	void Draw(); // const;

	EngineWindow win;
	Mesh test;

	bgfx::ProgramHandle shader = BGFX_INVALID_HANDLE;

	bx::Ticks lastFrame{bx::InitNone};
	float deltaTime{0.0f};

	Matrix<4> modelMat{Matrix<4>::Identity()};
	bool spin{true};
};

#ifdef __EMSCRIPTEN__
void WebLoop(void* arg);
#endif // __EMSCRIPTEN__
