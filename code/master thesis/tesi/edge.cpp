#include "edge.h"
#include "node.h"
namespace Graph
{
	Edge::Edge() :
		source{ nullptr },
		destination{ nullptr },
		distance{ 0 }
	{
	}
	Edge::Edge(Node* source, Node* destination, int distance) :
		source{ source },
		destination{ destination },
		distance{ distance }
	{
	}
}
