#pragma once

#include <cstdint>
#include <memory>
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

class Collider;
using Col_Sptr = std::shared_ptr<Collider>;

struct HitObj;
/** Type representing a range of float values between a min and a max value. */
struct Range
{
	float min;
	float max;
};
struct Edge
{
	uint32_t a;
	uint32_t b;
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
	[[nodiscard]] virtual vector<Col_Sptr>
	GetTransformed(const Matrix /*unused*/) const
	{
		return {};
	}
	/**
	 * Gets the relevant normals for doing the Separating Axis Theorem to check
	 * collisions and overlap.
	 */
	virtual void GetNormals(vector<Vector3>& /*unused*/) const = 0;
	/**
	 * Projects a collider along a normal axis and returns a Range representing
	 * the 'shadow' covered.
	 */
	[[nodiscard]] virtual Range GetProjection(const Vector3 /*nor*/) const
	{
		return {.min = 0.0f, .max = 0.0f};
	}

	virtual void DebugDraw(const Matrix& /*unused*/,
						   const Color& /*unused*/) const {};

	private:
};

/**
 * Special type of collider that collects several more simple colliders
 * together to create complex concave shapes
 */
class CompoundCollider : public Collider
{
	public:
	CompoundCollider(const vector<Col_Sptr>& cols);

	/** \copydoc Collider::GetTransformed() */
	[[nodiscard]] vector<Col_Sptr>
	GetTransformed(const Matrix trans) const override;
	/** \copydoc Collider::GetNormals() */
	void GetNormals(vector<Vector3>& out) const override;

	void DebugDraw(const Matrix& transform, const Color& col) const override;

	private:
	vector<Col_Sptr> colliders;
};

/** Convex Hull collider */
class HullCollider : public Collider
{
	public:
	HullCollider(const vector<Vector3>& verts, const vector<Edge>& edges,
				 const vector<Vector3>& nors);

	/** \copydoc Collider::GetTransformed() */
	[[nodiscard]] vector<Col_Sptr>
	GetTransformed(const Matrix trans) const override;
	/** \copydoc Collider::GetNormals() */
	void GetNormals(vector<Vector3>& out) const override;
	/** \copydoc Collider::GetProjection() */
	[[nodiscard]] Range GetProjection(const Vector3 nor) const override;

	/** Apply a transformation matrix to the Collider. */
	HullCollider operator*(const Matrix& mat);

	friend void GetEdgeCrosses(const std::shared_ptr<HullCollider> col1,
							   const std::shared_ptr<HullCollider> col2,
							   vector<Vector3>& out);

	void DebugDraw(const Matrix& transform, const Color& col) const override;

	private:
	vector<Edge> edges;
	vector<Vector3> vertices;
	vector<Vector3> normals;
};

///**
// * Checks if 2 colliders are overlapping
// *
// * \param col1 The first collider to check
// * \param trans1 The transform of the object associated with col1
// * \param col2 The first collider to check
// * \param trans2 The transform of the object associated with col1
// *
// * \returns A Hit Object struct if the colliders overlap, otheriwse returns
// * nothing.
// */
//std::optional<HitObj> CheckCollision(const Col_Sptr col1, const Matrix trans1,
//									 const Col_Sptr col2, const Matrix trans2);

/** Creates a rectangular convex hull collider centered on (0, 0, 0). */
std::shared_ptr<HullCollider> CreateBoxCollider(Matrix transform);

#ifndef NDEBUG
ostream& operator<<(ostream& ostr, HitObj hit);
ostream& operator<<(ostream& ostr, Vector3 vec);
ostream& operator<<(ostream& ostr, Range range);
ostream& operator<<(ostream& ostr, Matrix mat);
ostream& operator<<(ostream& ostr, Quaternion quat);
#endif // !NDEBUG

} //namespace phys
