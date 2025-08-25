#pragma once

#include <cstdint>
#include <ostream>
#include <raylib.h>
#include <vector>

namespace HE
{

struct HVertex;
struct HEdge;
struct HFace;

struct HVertex
{
	float x;
	float y;
	float z;
	uint8_t edgeID{0};

	[[nodiscard]] HEdge* Edge() const;
	[[nodiscard]] Vector3 Vec() const { return {this->x, this->y, this->z}; }

	HVertex& operator*(const Matrix mat)
	{
		Vector3 result;
		result.x = mat.m0 * x + mat.m4 * y + mat.m8 * z + mat.m12;
		result.y = mat.m1 * x + mat.m5 * y + mat.m9 * z + mat.m13;
		result.z = mat.m2 * x + mat.m6 * y + mat.m10 * z + mat.m14;
		this->x = result.x;
		this->y = result.y;
		this->z = result.z;
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

#ifndef NDEBUG
std::ostream& operator<<(std::ostream& ostr, HVertex vert);
std::ostream& operator<<(std::ostream& ostr, HEdge edge);
std::ostream& operator<<(std::ostream& ostr, HFace face);
#endif // !NDEBUG

} //namespace HE
