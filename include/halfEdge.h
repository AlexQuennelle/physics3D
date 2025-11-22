#pragma once

#include <cstdint>
#include <ostream>
#include <raylib.h>
#include <raymath.h>
#include <vector>

namespace HE
{

struct HVertex;
struct HEdge;
struct HFace;

struct Plane
{
	Vector3 pos{};
	Vector3 nor{};
};
inline auto IsPointBehindPlane(Plane plane, Vector3 point) -> bool
{
	return Vector3DotProduct(plane.nor, point - plane.pos) > 0;
}

struct HVertex
{
	float x{0.0f};
	float y{0.0f};
	float z{0.0f};
	uint8_t edgeID{0};

	auto Edge() const -> HEdge*;
	auto Vec() const -> Vector3 { return {this->x, this->y, this->z}; }

	void SetPos(const Vector3 newPos)
	{
		this->x = newPos.x;
		this->y = newPos.y;
		this->z = newPos.z;
	}

	auto operator*(const Matrix mat) -> HVertex&
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
	auto operator==(const HVertex& comp) const -> bool
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

	auto Vertex() const -> HVertex*;
	auto Twin() const -> HEdge*;
	auto Next() const -> HEdge*;
	auto Face() const -> HFace*;
	auto Dir() const -> Vector3;
	auto Center() const -> Vector3;
	auto Length() const -> float;

	std::vector<HE::HVertex>* vertArr{nullptr};
	std::vector<HEdge>* edgeArr{nullptr};
	std::vector<HE::HFace>* faceArr{nullptr};

	class Iterator
	{
		public:
		Iterator(HEdge* edge) : current(edge->Next()), start(edge) {}

		auto operator*() const -> HEdge& { return *current; }
		auto operator++() -> Iterator&
		{
			if (this->current != this->start)
				this->current = this->current->Next();
			else
				this->current = nullptr;
			return *this;
		}
		auto operator++(int) -> Iterator
		{
			Iterator temp = *this;
			++(*this);
			return temp;
		}
		auto operator==(const Iterator& other) const -> bool
		{
			if (this->current == nullptr && other.current == nullptr)
				return true;

			return this->current == other.current &&
				   (this->current != this->start ||
					other.current != this->start);
		}
		auto operator!=(const Iterator& other) const -> bool
		{
			return !(*this == other);
		}

		static auto EndIter() -> Iterator { return {nullptr, nullptr}; }

		private:
		Iterator(HEdge* edge, HEdge* start) : current(edge), start(start) {}

		HEdge* current;
		HEdge* start;
	};
	auto begin() -> Iterator { return {this}; }
	static auto end() -> Iterator { return Iterator::EndIter(); }
};

struct HFace
{

	HFace(const Vector3 nor) : normal(nor) {};
	Vector3 normal;
	uint8_t edgeID{0};

	auto Edge() const -> HEdge*;
	auto Center() const -> Vector3;
	auto Plane() const -> Plane;

	auto begin() -> HEdge::Iterator { return {this->Edge()}; }
	auto begin() const -> HEdge::Iterator { return {this->Edge()}; }
	static auto end() -> HEdge::Iterator { return HEdge::Iterator::EndIter(); }

	std::vector<HE::HEdge>* edgeArr{nullptr};
};

/** @brief Interface type for initializing half-edge structures with N sided
 *         faces.
 */
struct FaceInit
{
	Vector3 normal;
	std::vector<uint8_t> indices;
};

#ifndef NDEBUG
auto operator<<(std::ostream& ostr, HVertex vert) -> std::ostream&;
auto operator<<(std::ostream& ostr, HEdge edge) -> std::ostream&;
auto operator<<(std::ostream& ostr, HFace face) -> std::ostream&;
#endif // !NDEBUG

} //namespace HE
