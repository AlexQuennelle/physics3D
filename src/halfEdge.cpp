#include "halfEdge.h"

#include <ostream>

namespace HE
{

HEdge* HVertex::Edge() const { return &(*edgeArr)[this->edgeID]; }

HVertex* HEdge::Vertex() const { return &(*vertArr)[this->vertID]; }
HEdge* HEdge::Twin() const { return &(*edgeArr)[this->twinID]; }
HEdge* HEdge::Next() const { return &(*edgeArr)[this->nextID]; }
HFace* HEdge::Face() const { return &(*faceArr)[this->faceID]; }

HEdge* HFace::Edge() const { return &(*edgeArr)[this->edgeID]; }

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
