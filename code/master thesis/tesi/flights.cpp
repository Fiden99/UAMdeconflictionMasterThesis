#include "flights.h"
#include "node.h"

namespace Graph
{
	Flight::Flight() :
		source{ nullptr },
		destination{ nullptr }
	{
	}

	Flight::Flight(Node* source, Node* destination) :
		source{ source },
		destination{ destination }
	{
	}


}