#pragma once

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
	virtual const HitObj CheckCollision();

	private:
};

/**
 * Special type of collider that collects several more simple colliders together
 * to create complex concave shapes
 */
class CompoundCollider : public Collider
{
	public:
	CompoundCollider();

	private:
	vector<Collider> colliders;
};

/**
 * Polygonal Mesh collider
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
	bool Hit{false};
	Vector3 HitPos{0.0f, 0.0f, 0.0f};
};

/**
 * Creates a rectangular mesh collider with one corner at (0, 0, 0).
 * @param transform A transformation matrix.
 */
MeshCollider CreateBoxCollider(Matrix transform);

#ifndef NDEBUG
ostream operator<<(const ostream& ostr, HitObj hit);
#endif // !NDEBUG

} //namespace phys
