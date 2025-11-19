#pragma once

#include "collider.h"

#include <optional>
#include <raylib.h>
#include <raymath.h>
#include <variant>

namespace phys
{

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
	PhysObject(const Vector3 pos, const Mesh mesh, Col col);
	/**
	 * \param pos The initial position of the object in 3D space.
	 * \param mesh The mesh to render when Draw() is called.
	 * \param col The Collider to use for physics calculations.
	 * \param shader A shader to apply when rendering the mesh.
	 */
	PhysObject(const Vector3 pos, const Mesh mesh, Col col,
			   const Shader& shader);
	/**
	 * \param pos The initial position of the object in 3D space.
	 * \param mesh The mesh to render when Draw() is called.
	 * \param col The Collider to use for physics calculations.
	 * \param fragShader Path to the shader file to load.
	 * \param vertShader Path to the shader file to load.
	 */
	PhysObject(const Vector3 pos, const Mesh mesh, Col col,
			   const char* vertShader, const char* fragShader);

	void Update();
	void Draw() const;

	/**
	 * \returns The composite of the position, rotation, and scale
	 * transformation matrices.
	 */
	auto GetTransformM() const -> Matrix
	{
		return this->scale * this->rotation * this->position;
	}
	/** \returns The objects current position in world space. */
	auto GetPosition() const -> Vector3
	{
		return {position.m12, position.m13, position.m14};
	}
	/** \returns The object's current rotation in world space. */
	auto GetRotation() const -> Quaternion
	{
		Quaternion rot;
		Vector3 translation;
		Vector3 scale;
		MatrixDecompose(this->rotation, &translation, &rot, &scale);
		return rot;
	}
	auto GetScale() const -> Vector3
	{
		Quaternion rot;
		Vector3 translation;
		Vector3 scale;
		MatrixDecompose(this->scale, &translation, &rot, &scale);
		return scale;
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
	void SetScale(const float newScale)
	{
		this->scale = MatrixScale(newScale, newScale, newScale);
	}
	void SetScale(const Vector3 newScale)
	{
		this->scale = MatrixScale(newScale.x, newScale.y, newScale.z);
	}

	/** Rotates the object in world space. */
	void Rotate(const Quaternion& rot)
	{
		rotation = rotation * QuaternionToMatrix(rot);
	}

	/** \returns A pointer to the object's physics Collider. */
	auto GetCollider() const -> Col { return this->collider; }
	void GetColliderT(vector<Col>& out) const
	{
		// collider.GetTransformed(this->GetTransformM(), out);
		std::visit([this, &out](isCollider auto& col) -> void
		{
			col.GetTransformed(this->GetTransformM(), out);
		}, this->collider);
	}
	void GetColliderT(Matrix trans, vector<Col>& out) const
	{
		// collider->GetTransformed(trans, out);
		std::visit([&out, trans](isCollider auto& col) -> void
		{
			col.GetTransformed(trans, out);
		}, this->collider);
	}
	/** Sets the shader to use when drawing the object. */
	void SetShader(const Shader& newShader)
	{
		this->shader = newShader;
		this->material.shader = this->shader;
	}

	friend void DisplayObjectInfo(PhysObject& obj);

	private:
	Vector3 velocity;

	Matrix position;
	Matrix rotation;
	Matrix scale;

	Material material;
	Shader shader;

	Col collider;
	Mesh mesh;
};

// NOTE: This struct needs to be reworked
/** Struct representing a collision between colliders. */
struct HitObj
{
	public:
	Vector3 HitPos{0.0f, 0.0f, 0.0f};
	const PhysObject& ThisCol;
	const PhysObject& OtherCol;
};
struct RaycastHit
{
	float hitDist;
	Vector3 hitPos;
	PhysObject& hitObj;
};

auto CheckCollision(const PhysObject& obj1, const PhysObject& obj2)
	-> std::optional<HitObj>;
auto CheckRaycast(const Ray ray, PhysObject& obj) -> std::optional<RaycastHit>;

auto CreateBoxObject(const Vector3 pos, const Vector3 dims) -> PhysObject;

} //namespace phys
