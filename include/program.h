#pragma once

#include "physObject.h"

#include <imgui.h>
#include <raylib.h>
#include <vector>

namespace phys
{

using std::vector;

class Program
{
	public:
	Program();

	void Update();

	private:
	void ProcessInput();

	float deltaTime;
	float gravity{1.0f};
	vector<PhysObject> objects;
	Camera cam;

	PhysObject* selectedObj{nullptr};

	ImGuiIO& imguiIO;

	// NOTE: Remove when creating physics objects from meshes is properly
	// implemented.
	void DebugAddStairObj(Vector3 pos);
};
void DrawGrid(const float lineLength, const int count);

} //namespace phys
