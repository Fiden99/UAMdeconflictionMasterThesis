#pragma once
#include "node.h"

namespace Graph
{
	class Flight
	{
	public:
		Node* source;
		Node* destination;
		static int safetyDistance;
		Flight();
		Flight(Node* source, Node* destination);
	};
}