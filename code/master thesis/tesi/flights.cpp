#include "flights.h"
#include "node.h"

namespace Graph
{
	int Flight::safetyDistance = 0;
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
	Flight::Flight(Node* source, Node* destination, std::vector<int> earliestHatTime, std::vector<int> latestHatTime) :
		source{ source },
		destination{ destination },
		earliestHatTime{ earliestHatTime },
		latestHatTime{ latestHatTime }
	{
	}



	Flight::Flight(Node* source, Node* destination, int nNodes) :
		source{ source },
		destination{ destination },
		earliestHatTime(nNodes),
		latestHatTime(nNodes)
	{
	}
}