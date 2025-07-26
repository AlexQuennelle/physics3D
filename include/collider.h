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

class Collider
{
	public:
	private:
};

/** Special type of collider that collects several more simple colliders
 * together to create complex concave shapes
 */
class CompoundCollider : public Collider
{
	public:
	CompoundCollider();

	private:
	vector<Collider> colliders;
};

/** Polygonal Mesh collider
 */
class MeshCollider : public Collider
{
	public:
	MeshCollider(const vector<Vector3>& verts, const vector<Vector3>& nors);

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

struct HitObj
{
	public:
	const Collider& OtherCol;
	const Collider& ThisCol;
	Vector3 HitPos{0.0f, 0.0f, 0.0f};
};

std::optional<HitObj> CheckCollision(const Collider col1, const Matrix trans1,
									 const Collider col2, const Matrix trans2);

/** Creates a rectangular mesh collider with one corner at (0, 0, 0).
 * @param transform A transformation matrix.
 */
MeshCollider CreateBoxCollider(Matrix transform);

#ifndef NDEBUG
ostream& operator<<(ostream& ostr, HitObj hit);
ostream& operator<<(ostream& ostr, Vector3 vec);
#endif // !NDEBUG

} //namespace phys
