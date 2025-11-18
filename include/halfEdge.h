#pragma once

#include <cstdint>
#include <iterator>
#include <ostream>
#include <raylib.h>
#include <raymath.h>
#include <vector>

namespace HE
{

struct HVertex;
struct HEdge;
struct HFace;

struct HVertex
{
	// HVertex(const float x, const float y, const float z, uint8_t edge)
	// 	: x(x), y(y), z(z), edgeID(edge) {};
	// HVertex(const Vector3 vec, uint8_t edge)
	// 	: HVertex(vec.x, vec.y, vec.z, edge) {};

	float x;
	float y;
	float z;
	uint8_t edgeID{0};

	HEdge* Edge() const;
	Vector3 Vec() const { return {this->x, this->y, this->z}; }

	void SetPos(const Vector3 newPos)
	{
		this->x = newPos.x;
		this->y = newPos.y;
		this->z = newPos.z;
	}

	HVertex& operator*(const Matrix mat)
	{
		Vector3 result;
		result.x = (mat.m0 * x) + (mat.m4 * y) + (mat.m8 * z) + mat.m12;
		result.y = (mat.m1 * x) + (mat.m5 * y) + (mat.m9 * z) + mat.m13;
		result.z = (mat.m2 * x) + (mat.m6 * y) + (mat.m10 * z) + mat.m14;
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

	HVertex* Vertex() const;
	HEdge* Twin() const;
	HEdge* Next() const;
	HFace* Face() const;
	Vector3 Dir() const;
	Vector3 Center() const;
	float Length() const;

	std::vector<HE::HVertex>* vertArr{nullptr};
	std::vector<HEdge>* edgeArr{nullptr};
	std::vector<HE::HFace>* faceArr{nullptr};

	class Iterator
	{
		public:
		Iterator(HEdge* edge) : current(edge->Next()), start(edge) {}

		HEdge& operator*() const { return *current; }
		Iterator& operator++()
		{
			if (this->current != this->start)
				this->current = this->current->Next();
			else
				this->current = nullptr;
			return *this;
		}
		Iterator operator++(int)
		{
			Iterator temp = *this;
			++(*this);
			return temp;
		}
		bool operator==(const Iterator& other) const
		{
			if (this->current == nullptr && other.current == nullptr)
				return true;

			return this->current == other.current &&
				   (this->current != this->start ||
					other.current != this->start);
		}
		bool operator!=(const Iterator& other) const
		{
			return !(*this == other);
		}

		static Iterator EndIter() { return {nullptr, nullptr}; }

		private:
		Iterator(HEdge* edge, HEdge* start) : current(edge), start(start) {}

		HEdge* current;
		HEdge* start;
	};
	Iterator begin() { return {this}; }
	static Iterator end() { return Iterator::EndIter(); }
};

struct HFace
{

	HFace(const Vector3 nor) : normal(nor) {};
	Vector3 normal;
	uint8_t edgeID{0};

	HEdge* Edge() const;
	Vector3 Center() const;

	HEdge::Iterator begin() { return {this->Edge()}; }
	HEdge::Iterator begin() const { return {this->Edge()}; }
	static HEdge::Iterator end() { return HEdge::Iterator::EndIter(); }

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
