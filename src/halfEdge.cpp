#include "halfEdge.h"

namespace HE
{

HEdge* HVertex::Edge() const { return &(*edgeArr)[this->edgeID]; }

HVertex* HEdge::Vertex() const { return &(*vertArr)[this->twinID]; }
HEdge* HEdge::Twin() const { return &(*edgeArr)[this->twinID]; }
HEdge* HEdge::Next() const { return &(*edgeArr)[this->nextID]; }
HFace* HEdge::Face() const { return &(*faceArr)[this->faceID]; }

HEdge* HFace::Edge() const { return &(*edgeArr)[this->edgeID]; }

} //namespace HE
