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
	PhysObject(const Vector3 pos, const Mesh mesh, Collider* col,
			   const Shader& shader);

	void Update();
	void Draw() const;

	[[nodiscard]] Matrix GetTransformM() const
	{
		return MatrixMultiply(MatrixMultiply(scale, rotation), position);
	}
	[[nodiscard]] Vector3 GetPosition() const
	{
		return {position.m12, position.m13, position.m14};
	}
	[[nodiscard]] Quaternion GetRotation() const
	{
		Quaternion rot;
		Vector3 translation;
		Vector3 scale;
		MatrixDecompose(this->rotation, &translation, &rot, &scale);
		return rot;
	}

	void SetPosition(const Vector3& newPos)
	{
		position.m12 = newPos.x;
		position.m13 = newPos.y;
		position.m14 = newPos.z;
	}
	void SetRotation(const Matrix& newRot) { this->rotation = newRot; }
	void SetRotation(const Quaternion& newRot)
	{
		this->rotation = QuaternionToMatrix(newRot);
	}

	void Rotate(const Quaternion& rot)
	{
		rotation = rotation * QuaternionToMatrix(rot);
	}

	[[nodiscard]] Collider* GetCollider() const { return this->collider; }
	void SetShader(const Shader& newShader)
	{
		this->shader = newShader;
		this->material.shader = this->shader;
	}

	void SetShaderCol(Vector4 color) const
	{
		SetShaderValue(this->shader, this->ColLoc, &color, SHADER_UNIFORM_VEC4);
	}

	private:
	Vector3 velocity;

	Matrix position;
	Matrix rotation;
	Matrix scale;

	Material material;
	Shader shader;

	Collider* collider;
	Mesh mesh;

	//debug stuff
	int ColLoc;
};

PhysObject CreateBoxObject(const Vector3 pos, const Vector3 dims);

} //namespace phys
