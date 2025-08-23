#pragma once

#include <cstdint>
#include <raylib.h>
#include <vector>

namespace HE
{

struct HVertex;
struct HEdge;
struct HFace;

struct HVertex
{
	//Vertex(float x, float y, float z, std::vector<Edge>* arr)
	//	: x(x), y(y), z(z), edgeArr(arr) {};
	//Vertex(Vector3 pos, std::vector<Edge>* arr)
	//	: x(pos.x), y(pos.y), z(pos.z), edgeArr(arr) {};
	float x;
	float y;
	float z;
	uint8_t edgeID{0};

	[[nodiscard]] HEdge* Edge() const;
	[[nodiscard]] Vector3 Vec() const { return {this->x, this->y, this->z}; }

	HVertex& operator*(const Matrix mat)
	{
		this->x =
			mat.m0 * this->x + mat.m4 * this->y + mat.m8 * this->z + mat.m12;
		this->x =
			mat.m1 * this->x + mat.m5 * this->y + mat.m9 * this->z + mat.m13;
		this->x =
			mat.m2 * this->x + mat.m6 * this->y + mat.m10 * this->z + mat.m14;
		return *this;
	};
	bool operator==(const HVertex& comp) const
	{
		return this->x == comp.x && this->y == comp.y && this->z == comp.z;
	}

	std::vector<HE::HEdge>* edgeArr{nullptr};
};

struct HEdge
{
	//Edge(std::vector<Vertex>* verts, std::vector<Edge>* edges,
	//	 std::vector<Face>* faces)
	//	: vertArr(verts), edgeArr(edges), faceArr(faces) {};
	uint8_t vertID{0};
	uint8_t twinID{0};
	uint8_t nextID{0};
	uint8_t faceID{0};

	[[nodiscard]] HVertex* Vertex() const;
	[[nodiscard]] HEdge* Twin() const;
	[[nodiscard]] HEdge* Next() const;
	[[nodiscard]] HFace* Face() const;

	std::vector<HE::HVertex>* vertArr{nullptr};
	std::vector<HEdge>* edgeArr{nullptr};
	std::vector<HE::HFace>* faceArr{nullptr};
};

struct HFace
{

	//Face(Vector3 nor, std::vector<Edge>* arr) : normal(nor), edgeArr(arr) {};
	HFace(const Vector3 nor) : normal(nor) {};
	Vector3 normal;
	uint8_t edgeID{0};

	[[nodiscard]] HEdge* Edge() const;

	std::vector<HE::HEdge>* edgeArr{nullptr};
};

/** Interface type for initializing half-edge structures with N sided faces. */
struct FaceInit
{
	Vector3 normal;
	std::vector<uint8_t> indices;
};

} //namespace HE
