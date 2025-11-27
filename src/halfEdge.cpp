#include "halfEdge.h"

#include <ostream>
#include <raymath.h>

namespace HE
{

auto HVertex::Edge() const -> HEdge* { return &(*edgeArr)[this->edgeID]; }

auto HEdge::Vertex() const -> HVertex* { return &(*vertArr)[this->vertID]; }
auto HEdge::Twin() const -> HEdge* { return &(*edgeArr)[this->twinID]; }
auto HEdge::Next() const -> HEdge* { return &(*edgeArr)[this->nextID]; }
auto HEdge::Face() const -> HFace* { return &(*faceArr)[this->faceID]; }
auto HEdge::Dir() const -> Vector3
{
	return Vector3Normalize(this->Next()->Vertex()->Vec() - Vertex()->Vec());
}
auto HEdge::Center() const -> Vector3
{
	return (this->Vertex()->Vec() + this->Next()->Vertex()->Vec()) / 2;
}
auto HEdge::Length() const -> float
{
	return Vector3Length(this->Vertex()->Vec() - this->Next()->Vertex()->Vec());
}

auto HFace::Edge() const -> HEdge* { return &(*edgeArr)[this->edgeID]; }
auto HFace::Center() const -> Vector3
{
	Vector3 acc{};
	float count{0};
	for (const auto& edge : *this)
	{
		acc = acc + edge.Vertex()->Vec();
		count++;
	}
	return acc / count;
}
auto HFace::Plane() const -> struct Plane
{
	return {.pos = this->Edge()->Vertex()->Vec(), .nor = this->normal};
}

#ifndef NDEBUG
auto operator<<(std::ostream& ostr, HVertex vert) -> std::ostream&
{
	ostr << std::format("({:.3f}, {:.3f}, {:.3f})", vert.x, vert.y, vert.z);
	// ostr << '(' << vert.x << ", " << vert.y << ", " << vert.z << ')';
	return ostr;
}
auto operator<<(std::ostream& ostr, HEdge edge) -> std::ostream&
{
	ostr << *edge.Vertex() << "-->" << *edge.Twin()->Vertex();
	return ostr;
}
auto operator<<(std::ostream& ostr, HFace face) -> std::ostream&
{
	ostr
		<< '('
		<< face.normal.x
		<< ", "
		<< face.normal.y
		<< ", "
		<< face.normal.z
		<< ")\t";
	for (const auto& edge : face)
	{
		ostr << *edge.Vertex();
	}
	return ostr;
}
#endif // !NDEBUG

} //namespace HE
