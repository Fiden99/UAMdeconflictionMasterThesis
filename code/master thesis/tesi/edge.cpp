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
	Edge::Edge(Node* source, Node* destination, int distance, std::vector<double> v_min, std::vector<double> v_max) :
		source{ source },
		destination{ destination },
		distance{ distance },
		v_min{ v_min },
		v_max{ v_max }
	{
	}
	Edge::Edge(Node* source, Node* destination, int distance, int nFlights) :
		source{ source },
		destination{ destination },
		distance{ distance },
		v_min(nFlights),
		v_max(nFlights)
	{
	}
}
