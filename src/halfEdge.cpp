#include "halfEdge.h"
#include <iostream>

namespace HE
{

HEdge* HVertex::Edge() const { return &(*edgeArr)[this->edgeID]; }

HVertex* HEdge::Vertex() const
{
	if (this->vertArr == nullptr)
	{
		std::cout << "vertArr is null in edge\n";
	}
	return &(*vertArr)[this->vertID];
}
HEdge* HEdge::Twin() const
{
	if (this->edgeArr == nullptr)
	{
		std::cout << "edgeArr is null in edge\n";
	}
	return &(*edgeArr)[this->twinID];
}
HEdge* HEdge::Next() const
{
	if (this->edgeArr == nullptr)
	{
		std::cout << "edgeArr is null in edge\n";
	}
	return &(*edgeArr)[this->nextID];
}
HFace* HEdge::Face() const
{
	if (this->faceArr == nullptr)
	{
		std::cout << "faceArr is null in edge\n";
	}
	return &(*faceArr)[this->faceID];
}

HEdge* HFace::Edge() const { return &(*edgeArr)[this->edgeID]; }

} //namespace HE
