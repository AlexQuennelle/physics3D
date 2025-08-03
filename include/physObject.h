#pragma once

#include "collider.h"

#include <raylib.h>
#include <raymath.h>

namespace phys
{

class PhysObject
{
	public:
	PhysObject(const Vector3 pos, const Mesh mesh, Collider* col);

	[[nodiscard]] Matrix GetTransformM() const
	{
		return MatrixMultiply(MatrixMultiply(scale, rotation), position);
	}
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

	Collider* collider;
	Mesh mesh;

	private:
	Vector3 velocity;

	Matrix position;
	Matrix rotation;
	Matrix scale;

};

PhysObject CreateBoxObject(const Vector3 pos, const Vector3 dims);

} //namespace phys
