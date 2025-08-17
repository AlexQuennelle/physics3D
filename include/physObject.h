#pragma once

#include "collider.h"

#include <raylib.h>
#include <raymath.h>

namespace phys
{

extern vector<Shader> shaders;

/**
 * Object that interacts with the physics simulation systems. Has a collider for
 * collision detection and resolution, and a mesh and material for rendering.
 *
 * Physics and other logic happens in the Update() method, while rendering
 * happens in the Draw() method.
 */
class PhysObject
{
	public:
	/**
	 * \param pos The initial position of the object in 3D space.
	 * \param mesh The mesh to render when Draw() is called.
	 * \param col The Collider to use for physics calculations.
	 */
	PhysObject(const Vector3 pos, const Mesh mesh, Col_Uptr col);
	/**
	 * \param pos The initial position of the object in 3D space.
	 * \param mesh The mesh to render when Draw() is called.
	 * \param col The Collider to use for physics calculations.
	 * \param shader A shader to apply when rendering the mesh.
	 */
	PhysObject(const Vector3 pos, const Mesh mesh, Col_Uptr col,
			   const Shader& shader);
	/**
	 * \param pos The initial position of the object in 3D space.
	 * \param mesh The mesh to render when Draw() is called.
	 * \param col The Collider to use for physics calculations.
	 * \param fragShader Path to the shader file to load.
	 * \param vertShader Path to the shader file to load.
	 */
	PhysObject(const Vector3 pos, const Mesh mesh, Col_Uptr col,
			   const char* vertShader, const char* fragShader);

	void Update();
	void Draw() const;

	/**
	 * \returns The composite of the position, rotation, and scale
	 * transformation matrices.
	 */
	[[nodiscard]] Matrix GetTransformM() const
	{
		return MatrixMultiply(MatrixMultiply(scale, rotation), position);
	}
	/** \returns The objects current position in world space. */
	[[nodiscard]] Vector3 GetPosition() const
	{
		return {position.m12, position.m13, position.m14};
	}
	/** \returns The object's current rotation in world space. */
	[[nodiscard]] Quaternion GetRotation() const
	{
		Quaternion rot;
		Vector3 translation;
		Vector3 scale;
		MatrixDecompose(this->rotation, &translation, &rot, &scale);
		return rot;
	}

	/** Sets the object's position in world space. */
	void SetPosition(const Vector3& newPos)
	{
		position.m12 = newPos.x;
		position.m13 = newPos.y;
		position.m14 = newPos.z;
	}
	/** Sets the object's rotation in world space using a rotation Matrix. */
	void SetRotation(const Matrix& newRot) { this->rotation = newRot; }
	/** Sets the object's rotation in world space. */
	void SetRotation(const Quaternion& newRot)
	{
		this->rotation = QuaternionToMatrix(newRot);
	}

	/** Rotates the object in world space. */
	void Rotate(const Quaternion& rot)
	{
		rotation = rotation * QuaternionToMatrix(rot);
	}

	/** \returns A pointer to the object's physics Collider. */
	[[nodiscard]] const Collider& GetCollider() const
	{
		return *this->collider;
	}
	/** Sets the shader to use when drawing the object. */
	void SetShader(const Shader& newShader)
	{
		this->shader = newShader;
		this->material.shader = this->shader;
	}

	private:
	Vector3 velocity;

	Matrix position;
	Matrix rotation;
	Matrix scale;

	Material material;
	Shader shader;

	Col_Uptr collider;
	Mesh mesh;

	//debug stuff
	int ColLoc;
};

PhysObject CreateBoxObject(const Vector3 pos, const Vector3 dims);

} //namespace phys
