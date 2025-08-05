#pragma once

#include <optional>
#include <raylib.h>
#include <vector>
#ifndef NDEBUG
#include <ostream>
#endif // !NDEBUG

namespace phys
{

using std::vector;
#ifndef NDEBUG
using std::ostream;
#endif

struct HitObj;
/** Type representing a range of float values between a min and a max value. */
struct Range
{
	float min;
	float max;
};

/** Abstract Collider class */
class Collider
{
	// TODO: Add mass property
	public:
	virtual ~Collider() = default;
	/**
	 * Applies a transformation matrix to a Collider and returns it in a
	 * vector.
	 * \returns A vector of Colliders transformed by the supplied matrix. The
	 * Vector may contain multiple Colliders.
	 */
	[[nodiscard]] virtual vector<Collider*>
	GetTransformed(const Matrix /*trans*/) const
	{
		return {};
	}
	/**
	 * Gets the relevant normals for doing the Separating Axis Theorem to check
	 * collisions and overlap.
	 */
	[[nodiscard]] virtual vector<Vector3> GetNormals() const { return {}; }
	/**
	 * Projects a collider along a normal axis and returns a Range representing
	 * the 'shadow' covered.
	 */
	[[nodiscard]] virtual Range GetProjection(const Vector3 /*nor*/) const
	{
		return {.min = 0.0f, .max = 0.0f};
	}

	private:
};

/**
 * Special type of collider that collects several more simple colliders
 * together to create complex concave shapes
 */
class CompoundCollider : public Collider
{
	public:
	CompoundCollider(const vector<Collider*>& cols);

	/** \copydoc Collider::GetTransformed() */
	[[nodiscard]] vector<Collider*>
	GetTransformed(const Matrix trans) const override;
	/** \copydoc Collider::GetNormals() */
	[[nodiscard]] vector<Vector3> GetNormals() const override;

	private:
	vector<Collider*> colliders;
};

/** Polygonal Mesh collider */
class MeshCollider : public Collider
{
	public:
	MeshCollider(const vector<Vector3>& verts, const vector<Vector3>& nors);

	/** \copydoc Collider::GetTransformed() */
	[[nodiscard]] vector<Collider*>
	GetTransformed(const Matrix trans) const override;
	/** \copydoc Collider::GetNormals() */
	[[nodiscard]] vector<Vector3> GetNormals() const override;
	/** \copydoc Collider::GetProjection() */
	[[nodiscard]] Range GetProjection(const Vector3 nor) const override;

	/** Apply a transformation matrix to the Collider. */
	MeshCollider operator*(const Matrix& mat);

	vector<Vector3> vertices;
	vector<Vector3> normals;

	private:
};

// TODO: Implement or remove.
// This Collider type may be more effort than it's worth.
class SphereCollider : public Collider
{
	public:
	SphereCollider();

	private:
};

/** Struct representing a collision between colliders. */
struct HitObj
{
	public:
	const Collider* ThisCol;
	const Collider* OtherCol;
	Vector3 HitPos{0.0f, 0.0f, 0.0f};
};

/**
 * Checks if 2 colliders are overlapping
 *
 * \param col1 The first collider to check
 * \param trans1 The transform of the object associated with col1
 * \param col2 The first collider to check
 * \param trans2 The transform of the object associated with col1
 *
 * \returns A Hit Object struct if the colliders overlap, otheriwse returns
 * nothing.
 */
std::optional<HitObj> CheckCollision(const Collider* col1, const Matrix trans1,
									 const Collider* col2, const Matrix trans2);

/**
 * Creates a rectangular mesh collider centered on (0, 0, 0).
 */
MeshCollider* CreateBoxCollider(Matrix transform);

#ifndef NDEBUG
ostream& operator<<(ostream& ostr, HitObj hit);
ostream& operator<<(ostream& ostr, Vector3 vec);
ostream& operator<<(ostream& ostr, Range range);
ostream& operator<<(ostream& ostr, Matrix mat);
ostream& operator<<(ostream& ostr, Quaternion quat);
#endif // !NDEBUG

} //namespace phys
