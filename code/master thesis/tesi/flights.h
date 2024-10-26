#pragma once
#include "node.h"
#include <vector>

namespace Graph
{
	class Flight
	{
	public:
		Node* source;
		Node* destination;
		std::vector<int> earliestHatTime;
		std::vector<int> latestHatTime;
		static int safetyDistance;
		Flight();
		Flight(Node* source, Node* destination);
		Flight(Node* source, Node* destination, std::vector<int> earliestHatTime, std::vector<int> latestHatTime);
		Flight(Node* source, Node* destination, int nNodes);
	};
}