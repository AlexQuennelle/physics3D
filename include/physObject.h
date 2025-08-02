#pragma once

#include <raylib.h>

namespace phys
{

class PhysObject
{
	public:
	PhysObject();

	[[nodiscard]] Vector3 GetPosition() const
	{
		return {position.m12, position.m13, position.m14};
	}
	void SetPosition(const Vector3& newPos)
	{
		position.m12 = newPos.x;
		position.m13 = newPos.y;
		position.m14 = newPos.z;
	}

	private:
	Matrix position;
	Matrix rotation;
	Matrix scale;
	Vector3 velocity;
};

} //namespace phys
