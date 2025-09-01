#include "halfEdge.h"

#include <ostream>
#include <raymath.h>

namespace HE
{

HEdge* HVertex::Edge() const { return &(*edgeArr)[this->edgeID]; }

HVertex* HEdge::Vertex() const { return &(*vertArr)[this->vertID]; }
HEdge* HEdge::Twin() const { return &(*edgeArr)[this->twinID]; }
HEdge* HEdge::Next() const { return &(*edgeArr)[this->nextID]; }
HFace* HEdge::Face() const { return &(*faceArr)[this->faceID]; }
Vector3 HEdge::Dir() const
{
	return Vector3Normalize(Vertex()->Vec() - Next()->Vertex()->Vec());
}
Vector3 HEdge::Center() const
{
	return (this->Vertex()->Vec() + this->Next()->Vertex()->Vec()) / 2;
}
float HEdge::Length() const
{
	return Vector3Length(this->Vertex()->Vec() - this->Next()->Vertex()->Vec());
}

HEdge* HFace::Edge() const { return &(*edgeArr)[this->edgeID]; }
Vector3 HFace::Center() const
{
	auto* edge{this->Edge()};
	Vector3 acc;
	int count{0};
	do
	{
		edge = edge->Next();
		acc = acc + edge->Vertex()->Vec();
		count++;
	}
	while (edge->Vertex() != this->Edge()->Vertex());
	return acc / count;
}

#ifndef NDEBUG
std::ostream& operator<<(std::ostream& ostr, HVertex vert)
{
	ostr << '(' << vert.x << ", " << vert.y << ", " << vert.z << ')';
	return ostr;
}
std::ostream& operator<<(std::ostream& ostr, HEdge edge)
{
	ostr << *edge.Vertex() << "-->" << *edge.Twin()->Vertex();
	return ostr;
}
std::ostream& operator<<(std::ostream& ostr, HFace face)
{
	ostr << '(' << face.normal.x << ", " << face.normal.y << ", "
		 << face.normal.z << ")\t";
	auto* vert = face.Edge();
	do
	{
		vert = vert->Next();
		ostr << *vert->Vertex();
	}
	while (*face.Edge()->Vertex() != *vert->Vertex());
	return ostr;
}
#endif // !NDEBUG

} //namespace HE
