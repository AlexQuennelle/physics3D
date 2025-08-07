#pragma once

#include "physObject.h"

#include <raylib.h>
#include <vector>

namespace phys
{

using std::vector;

class World
{
	public:
	World();

	void Update();

	private:
	void UpdateCamera();

	float deltaTime;
	float gravity{1.0f};
	vector<PhysObject> objects;
	Camera cam;

#ifndef NDEBUG
	void DebugAddStairObj();
#endif // !NDEBUG
};
void DrawGrid(const float lineLength, const int count);

} //namespace phys
