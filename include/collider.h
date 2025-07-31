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

/** Abstract Collider class */
class Collider
{
	public:
	virtual ~Collider() = default;
	/** Applies a transformation matrix to a Collider and returns it in a
	 * vector.
	 * @returns A vector of Colliders transformed by the supplied matrix. The
	 * Vector may contain multiple Colliders.
	 */
	[[nodiscard]] virtual std::vector<Collider>
	GetTransformed(const Matrix  /*trans*/) const
	{
		return {};
	}

	private:
};

/** Special type of collider that collects several more simple colliders
 * together to create complex concave shapes
 */
class CompoundCollider : public Collider
{
	public:
	CompoundCollider();

	[[nodiscard]] vector<Collider>
	GetTransformed(const Matrix trans) const override;

	private:
	vector<Collider> colliders;
};

/** Polygonal Mesh collider */
class MeshCollider : public Collider
{
	public:
	MeshCollider(const vector<Vector3>& verts, const vector<Vector3>& nors);

	/** @copydoc Collider::GetTransformed()
	 */
	[[nodiscard]] vector<Collider>
	GetTransformed(const Matrix trans) const override;

	MeshCollider operator*(const Matrix mat);

	private:
	vector<Vector3> vertices;
	vector<Vector3> normals;
};

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
	const Collider& ThisCol;
	const Collider& OtherCol;
	Vector3 HitPos{0.0f, 0.0f, 0.0f};
};

/** Checks if 2 colliders are overlapping
 *
 * @params col1 The first collider to check
 * @params trans1 The transform of the object associated with col1
 * @params col2 The first collider to check
 * @params trans2 The transform of the object associated with col1
 *
 * @returns A Hit Object struct if the colliders overlap, otheriwse returns
 * nothing.
 */
std::optional<HitObj> CheckCollision(const Collider& col1, const Matrix trans1,
									 const Collider& col2, const Matrix trans2);

/** Creates a rectangular mesh collider with one corner at (0, 0, 0).
 * @param transform A transformation matrix.
 */
MeshCollider CreateBoxCollider(Matrix transform);

#ifndef NDEBUG
ostream& operator<<(ostream& ostr, HitObj hit);
ostream& operator<<(ostream& ostr, Vector3 vec);
#endif // !NDEBUG

} //namespace phys
