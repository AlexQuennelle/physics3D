#pragma once

#include "halfEdge.h"

#include <cstdint>
#include <memory>
#include <raylib.h>
#include <raymath.h>
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
struct RaycastHit;
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
	virtual void GetTransformed(const Matrix /*unused*/,
								vector<Col_Sptr>& /*unused*/) const = 0;

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

	[[nodiscard]] virtual Vector3
	GetSupportPoint(const Vector3& axis) const = 0;

	virtual void DebugDraw(const Matrix& /*unused*/,
						   const Color& /*unused*/) const {};

	struct FaceHit
	{
		uint8_t id;
		float penetration;
		Vector3 suppport;
	};
	struct EdgeHit
	{
		uint8_t id1;
		uint8_t id2;
		float penetration;
		Vector3 support;
		Vector3 normal;
	};
	friend FaceHit CheckFaceNors(Col_Sptr col1, Col_Sptr col2);
	friend EdgeHit CheckEdgeNors(Col_Sptr col1, Col_Sptr col2);

	protected:
	Vector3 origin{0.0f, 0.0f, 0.0f};
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
	void GetTransformed(const Matrix trans,
						vector<Col_Sptr>& out) const override;
	/** \copydoc Collider::GetNormals() */
	void GetNormals(vector<Vector3>& out) const override;

	[[nodiscard]] Vector3 GetSupportPoint(const Vector3& axis) const override;

	void DebugDraw(const Matrix& transform, const Color& col) const override;

	private:
	vector<Col_Sptr> colliders;
};

/** Convex Hull collider */
class HullCollider : public Collider
{
	public:
	HullCollider(const vector<HE::HVertex>& verts,
				 const vector<HE::FaceInit>& faces,
				 const Vector3 origin = Vector3Zero());
	HullCollider(const HullCollider& copy);

	/** \copydoc Collider::GetTransformed() */
	void GetTransformed(const Matrix trans,
						vector<Col_Sptr>& out) const override;
	/** \copydoc Collider::GetNormals() */
	void GetNormals(vector<Vector3>& out) const override;
	/** \copydoc Collider::GetProjection() */
	[[nodiscard]] Range GetProjection(const Vector3 nor) const override;

	/** Apply a transformation matrix to the Collider. */
	HullCollider operator*(const Matrix& mat);

	[[nodiscard]] Vector3 GetSupportPoint(const Vector3& axis) const override;
	[[nodiscard]] const HE::HFace& GetFace(const uint8_t i) const
	{
		return faces[i];
	}
	[[nodiscard]] uint8_t FaceCount() const { return this->faces.size(); }

	friend void GetEdgeCrosses(const std::shared_ptr<HullCollider> col1,
							   const std::shared_ptr<HullCollider> col2,
							   vector<Vector3>& out);

	void DebugDraw(const Matrix& transform, const Color& col) const override;

	friend FaceHit CheckFaceNors(Col_Sptr col1, Col_Sptr col2);
	friend EdgeHit CheckEdgeNors(Col_Sptr col1, Col_Sptr col2);

	private:
	vector<HE::HEdge> edges;
	vector<HE::HVertex> vertices;
	vector<HE::HFace> faces;
};

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
