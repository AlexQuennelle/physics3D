#include "program.h"
#include "backends/imgui_impl_glfw.h"
#include "matrix.h"
#include "mesh.h"

#include <imgui/imgui_impl_bgfx.h>

#include <GLFW/glfw3.h>
#include <bgfx/bgfx.h>
#include <bx/math.h>
#include <csignal>
#include <cstdint>
#include <imgui.h>
#include <print>
#ifndef __EMSCRIPTEN__
#include <GLFW/glfw3native.h>
#else
#include <emscripten/emscripten.h>
#endif // !__EMSCRIPTEN__

Program::Program()
{
	// TODO: Pull this out to init function
#ifndef __EMSCRIPTEN__
	if (glfwPlatformSupported(GLFW_PLATFORM_WAYLAND) != 0)
	{
		glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_WAYLAND);
		std::println("Wayland");
	}
	else if (glfwPlatformSupported(GLFW_PLATFORM_X11) != 0)
	{
		glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);
		std::println("X11");
		raise(SIGTRAP);
	}
#endif // !__EMSCRIPTEN__
	glfwWindowHint(GLFW_TRANSPARENT_FRAMEBUFFER, GLFW_FALSE);
	glfwInit();
	bgfx::init();
	this->win = EngineWindow(NAME, 800, 800);
}
Program::~Program()
{
	ImGui_ImplGlfw_Shutdown();
	ImGui_ImplBGFX_Shutdown();
	glfwTerminate();
}

void Program::Run()
{
	this->Init();

#ifdef __EMSCRIPTEN__
	emscripten_set_main_loop_arg(WebLoop, this, 0, 1);
#else
	while (!this->win.ShouldClose())
	{
		this->Update();
		this->Draw();
	}
	this->win.Close();
#endif // !__EMSCRIPTEN__
}

void Program::Init()
{
	// TODO: Put this in proper init function
	this->win.BeginContext();

	bgfx::Init init;
	init.vendorId = BGFX_PCI_ID_NONE;
#ifdef __EMSCRIPTEN__
	init.type = bgfx::RendererType::OpenGL;
#else
	std::println("{}", Backend);
	if (Backend == "VULKAN")
		init.type = bgfx::RendererType::Vulkan;
	else if (Backend == "OPEN_GL")
		init.type = bgfx::RendererType::OpenGL;
	else
		init.type = bgfx::RendererType::Noop;
#endif // __EMSCRIPTEN__

	init.platformData.nwh = this->win.GetNativeHandle();
#ifdef __linux__
	// raise(SIGTRAP);
	if (glfwPlatformSupported(GLFW_PLATFORM_WAYLAND) != 0)
	{
		init.platformData.ndt = glfwGetWaylandDisplay();
		init.platformData.type = bgfx::NativeWindowHandleType::Wayland;
	}
	else
	{
		init.platformData.ndt = glfwGetX11Display();
		init.platformData.type = bgfx::NativeWindowHandleType::Default;
	}
#else
	init.platformData.type = bgfx::NativeWindowHandleType::Default;
#endif // __linux__
	init.platformData.context = nullptr;

	init.resolution.width = static_cast<uint32_t>(this->win.GetHeight());
	init.resolution.height = static_cast<uint32_t>(this->win.GetWidth());
	init.resolution.reset = BGFX_RESET_VSYNC | BGFX_RESET_MSAA_X4;

	bgfx::init(init);

	lastFrame = bx::getNow();

	bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH, 0x00FF00FF, 1.0f,
					   0);

	// ImGui::CreateContext();
	ImGui_ImplBGFX_Init();
	ImGui_ImplGlfw_InitForVulkan(this->win.GetGLFWHandle(), true);
	// TODO: Add proper selection for backends

	Vertex::Init();
	this->modelMat = Matrix<4>::Identity();

	test = Mesh(RESOURCES_PATH "Suzanne.obj");
}
void Program::Update()
{
	auto spaceKey = glfwGetKey(this->win.GetGLFWHandle(), GLFW_KEY_SPACE);
	if (spaceKey == GLFW_PRESS)
		this->spin = !this->spin;
	// TODO: Move to time manager object
	this->deltaTime = static_cast<float>(
		static_cast<double>(bx::getNow().ticks - this->lastFrame.ticks)
		/ static_cast<double>(this->lastFrame.s_kFreq.ticks));
	this->lastFrame = bx::getNow();
	if (spin)
		this->modelMat = modelMat.RotateY(15.0f * this->deltaTime);
	// this->modelMat *= Matrix<4>::FromAngleY(5.0f * this->deltaTime);
}
void Program::Draw()
{
	// TODO: Add event processing API
	this->win.BeginContext();
	glfwPollEvents();
	// TODO: Fix this shit
	ImGui_ImplGlfw_NewFrame();
	ImGui_ImplBGFX_NewFrame();
	const Vector3 at = {.x = 0.0f, .y = 0.0f, .z = 0.0f};
	const Vector3 eye = {.x = 0.0f, .y = 2.0f, .z = -4.0f};

	ImGui::SetNextWindowPos({5.0f, 5.0f});
	bool showDemo{true};
	if (showDemo)
	{
		ImGui::ShowDemoWindow(&showDemo);
	}

	{
		Matrix viewMat{Matrix<4>::LookAt(eye, at)};

		Matrix projMat{Matrix<4>::Projection(
			60.0f,
			static_cast<float>(this->win.GetWidth())
				/ static_cast<float>(this->win.GetWidth()),
			0.1f, 100.0f, bgfx::getCaps()->homogeneousDepth)};
		bgfx::setViewTransform(0, viewMat.Data(), projMat.Data());

		// Set view 0 default viewport.
		bgfx::setViewRect(0, 0, 0, static_cast<uint16_t>(this->win.GetWidth()),
						  static_cast<uint16_t>(this->win.GetHeight()));
	}

	bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH, 0xFFFFFFFF, 1.0f,
					   0);
	bgfx::setViewRect(0, 0, 0, static_cast<uint16_t>(this->win.GetWidth()),
					  static_cast<uint16_t>(this->win.GetHeight()));
	// bgfx::touch(0);

	// auto modelMat{Matrix<4>::Identity()};
	auto testState{BGFX_STATE_WRITE_RGB
				   // | BGFX_STATE_WRITE_A
				   | BGFX_STATE_WRITE_Z
				   | BGFX_STATE_DEPTH_TEST_LESS
				   | BGFX_STATE_CULL_CCW
				   | BGFX_STATE_BLEND_ALPHA
				   | BGFX_STATE_MSAA};
	bgfx::setState(testState);
	// TODO: Add GameObject/Actor class to encapsulate this stuff
	bgfx::setTransform(modelMat.Data());
	this->test.Draw();
	// for (uint32_t yy = 0; yy < 11; ++yy)
	// {
	// 	for (uint32_t xx = 0; xx < 11; ++xx)
	// 	{
	// 		float mtx[16];
	// 		bx::mtxRotateXY(mtx, xx * 0.21f, yy * 0.37f);
	// 		mtx[12] = -15.0f + float(xx) * 3.0f;
	// 		mtx[13] = -15.0f + float(yy) * 3.0f;
	// 		mtx[14] = 0.0f;

	// 		// Set model matrix for rendering.
	// 		bgfx::setTransform(mtx);
	// 		this->test.Draw();
	// 	}
	// }
	// TODO: Wrap this behind some API
	ImGui_ImplBGFX_EndFrame();
	bgfx::frame();
}

#ifdef __EMSCRIPTEN__
void WebLoop(void* arg)
{
	auto* program = static_cast<Program*>(arg);
	program->Update();
	program->Draw();
}
#endif // __EMSCRIPTEN__
