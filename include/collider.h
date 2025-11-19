#pragma once

#include "halfEdge.h"

#include <concepts>
#include <cstdint>
#include <memory>
#include <raylib.h>
#include <raymath.h>
#include <variant>
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

class HullCollider;
class CompoundCollider;

using Col = std::variant<HullCollider, CompoundCollider>;

struct HitObj;
struct RaycastHit;
/** Type representing a range of float values between a min and a max value. */
struct Range
{
	float min;
	float max;
};
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

template <typename T>
concept isCollider = requires(const T a, const Matrix m, vector<Col>& vec,
							  const Vector3 p, vector<Vector3> nors, Color c) {
	{ a.GetTransformed(m, vec) } -> std::same_as<void>;
	{ a.GetNormals(nors) } -> std::same_as<void>;
	{ a.GetProjection(p) } -> std::same_as<Range>;
	{ a.GetSupportPoint(p) } -> std::same_as<Vector3>;
	{ a.DebugDraw(m, c) } -> std::same_as<void>;
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
								vector<Col>& /*unused*/) const = 0;

	/**
	 * Gets the relevant normals for doing the Separating Axis Theorem to check
	 * collisions and overlap.
	 */
	virtual void GetNormals(vector<Vector3>& /*unused*/) const = 0;
	/**
	 * Projects a collider along a normal axis and returns a Range representing
	 * the 'shadow' covered.
	 */
	virtual auto GetProjection(const Vector3 /*nor*/) const -> Range
	{
		return {.min = 0.0f, .max = 0.0f};
	}

	virtual auto GetSupportPoint(const Vector3& axis) const -> Vector3 = 0;

	virtual void DebugDraw(const Matrix& /*unused*/,
						   const Color& /*unused*/) const {};

	friend auto CheckFaceNors(Collider& colA, Collider& colB) -> FaceHit;
	friend auto CheckEdgeNors(Collider& colA, Collider& colB) -> EdgeHit;

	protected:
	Vector3 origin{0.0f, 0.0f, 0.0f};
};

static_assert(isCollider<Collider>);

/**
 * Special type of collider that collects several more simple colliders
 * together to create complex concave shapes
 */
class CompoundCollider : public Collider
{
	public:
	CompoundCollider(const vector<Col>& cols);

	/** \copydoc Collider::GetTransformed() */
	void GetTransformed(const Matrix trans, vector<Col>& out) const override;
	/** \copydoc Collider::GetNormals() */
	void GetNormals(vector<Vector3>& out) const override;

	auto GetSupportPoint(const Vector3& axis) const -> Vector3 override;

	void DebugDraw(const Matrix& transform, const Color& col) const override;

	private:
	vector<Col> colliders;
};
static_assert(isCollider<CompoundCollider>);

/** Convex Hull collider */
class HullCollider : public Collider
{
	public:
	HullCollider(const vector<HE::HVertex>& verts,
				 const vector<HE::FaceInit>& faces,
				 const Vector3 origin = Vector3Zero());
	HullCollider(const HullCollider& copy);

	/** \copydoc Collider::GetTransformed() */
	void GetTransformed(const Matrix trans, vector<Col>& out) const override;
	/** \copydoc Collider::GetNormals() */
	void GetNormals(vector<Vector3>& out) const override;
	/** \copydoc Collider::GetProjection() */
	auto GetProjection(const Vector3 nor) const -> Range override;

	/** Apply a transformation matrix to the Collider. */
	auto operator*(const Matrix& mat) -> HullCollider;

	auto GetSupportPoint(const Vector3& axis) const -> Vector3 override;
	auto GetFace(const uint8_t i) const -> const HE::HFace& { return faces[i]; }
	auto FaceCount() const -> uint8_t { return this->faces.size(); }

	friend void GetEdgeCrosses(const HullCollider& col1,
							   const HullCollider& col2, vector<Vector3>& out);

	void DebugDraw(const Matrix& transform, const Color& col) const override;

	friend auto CheckFaceNors(Col colA, Col colB) -> FaceHit;
	friend auto CheckEdgeNors(Col colA, Col colB) -> EdgeHit;

	private:
	vector<HE::HEdge> edges;
	vector<HE::HVertex> vertices;
	vector<HE::HFace> faces;
};
static_assert(isCollider<HullCollider>);

/** Creates a rectangular convex hull collider centered on (0, 0, 0). */
auto CreateBoxCollider(Matrix transform) -> Col;

#ifndef NDEBUG
auto operator<<(ostream& ostr, HitObj hit) -> ostream&;
auto operator<<(ostream& ostr, Vector3 vec) -> ostream&;
auto operator<<(ostream& ostr, Range range) -> ostream&;
auto operator<<(ostream& ostr, Matrix mat) -> ostream&;
auto operator<<(ostream& ostr, Quaternion quat) -> ostream&;
#endif // !NDEBUG

} //namespace phys
