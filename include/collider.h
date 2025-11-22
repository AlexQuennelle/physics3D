#pragma once

#include "halfEdge.h"

#include <concepts>
#include <cstdint>
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

using Collider = std::variant<HullCollider, CompoundCollider>;

struct HitObj;
struct RaycastHit;
/** @brief Type representing a range of float values between a min and a max
 *         value.
 */
struct Range
{
	float min;
	float max;
};
struct FaceHit
{
	uint8_t id;
	float penetration;
	Vector3 support;
};
struct EdgeHit
{
	uint8_t id1;
	uint8_t id2;
	float penetration;
	Vector3 support;
	Vector3 normal;
};

/** @brief Interface for convex collider types. Ensures compliance with a
 *         particular set of methods used by the collision detection systems.
 */
template <typename T>
concept isCollider
	= requires(const T col, const Matrix mat, vector<Collider>& arr,
			   const Vector3 vec, vector<Vector3> nors, Color color) {
		  { col.GetOrigin() } -> std::same_as<Vector3>;
		  { col.GetTransformed(mat, arr) } -> std::same_as<void>;
		  { col.GetNormals(nors) } -> std::same_as<void>;
		  // { col.GetProjection(vec) } -> std::same_as<Range>;
		  { col.GetSupportPoint(vec) } -> std::same_as<Vector3>;
		  { col.DebugDraw(mat, color) } -> std::same_as<void>;
	  };

/** @brief Special type of collider that collects several more simple colliders
 *         together to create complex concave shapes
 */
class CompoundCollider //: public Collider
{
	public:
	CompoundCollider(const vector<Collider>& cols);

	auto GetOrigin() const -> Vector3 { return this->origin; }
	void GetTransformed(const Matrix trans, vector<Collider>& out) const;
	void GetNormals(vector<Vector3>& out) const;

	static auto GetSupportPoint(const Vector3 axis) -> Vector3; // override;

	static auto GetProjection(const Vector3 /*nor*/) -> Range
	{
		return {.min = 0.0f, .max = 0.0f};
	}

	void DebugDraw(const Matrix& transform,
				   const Color& col) const; // override;

	private:
	vector<Collider> colliders;
	Vector3 origin{0.0f, 0.0f, 0.0f};
};
static_assert(isCollider<CompoundCollider>);

/** @brief Convex Hull collider */
class HullCollider
{
	public:
	HullCollider(const vector<HE::HVertex>& verts,
				 const vector<HE::FaceInit>& faces,
				 const Vector3 origin = Vector3Zero());
	HullCollider(const HullCollider& copy);
	HullCollider(HullCollider&&) = delete;
	~HullCollider() = default;

	auto operator=(const HullCollider&) -> HullCollider& = default;
	auto operator=(HullCollider&&) -> HullCollider& = delete;

	auto GetOrigin() const -> Vector3 { return this->origin; }
	void GetTransformed(const Matrix trans, vector<Collider>& out) const;
	void GetNormals(vector<Vector3>& out) const;
	auto GetProjection(const Vector3 nor) const -> Range;

	/** @brief Apply a transformation matrix to the Collider. */
	auto operator*(const Matrix& mat) -> HullCollider;

	auto GetSupportPoint(const Vector3 axis) const -> Vector3;
	auto GetFace(const uint8_t i) const -> const HE::HFace& { return faces[i]; }
	auto FaceCount() const -> uint8_t { return this->faces.size(); }

	friend void GetEdgeCrosses(const HullCollider& col1,
							   const HullCollider& col2, vector<Vector3>& out);

	void DebugDraw(const Matrix& transform, const Color& col) const;

	friend auto CheckFaceNors(Collider colA, Collider colB) -> FaceHit;
	friend auto CheckEdgeNors(Collider colA, Collider colB) -> EdgeHit;

	private:
	vector<HE::HEdge> edges;
	vector<HE::HVertex> vertices;
	vector<HE::HFace> faces;
	Vector3 origin{0.0f, 0.0f, 0.0f};
};
static_assert(isCollider<HullCollider>);

/** @brief Creates a rectangular convex hull collider centered on (0, 0, 0). */
auto CreateBoxCollider(Matrix transform) -> Collider;

#ifndef NDEBUG
auto operator<<(ostream& ostr, HitObj hit) -> ostream&;
auto operator<<(ostream& ostr, Vector3 vec) -> ostream&;
auto operator<<(ostream& ostr, Range range) -> ostream&;
auto operator<<(ostream& ostr, Matrix mat) -> ostream&;
auto operator<<(ostream& ostr, Quaternion quat) -> ostream&;
#endif // !NDEBUG

} //namespace phys
