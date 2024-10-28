#pragma once
#include <vector>
#include "node.h"
namespace Graph
{
	class Node;
	class Edge
	{
	public:
		Node* source;
		Node* destination;
		int distance;
		std::vector<double> v_min;
		std::vector<double> v_max;
		Edge();
		Edge(Node* source, Node* destination, int distance);
		Edge(Node* source, Node* destination, int distance, std::vector<double> v_min, std::vector<double> v_max);
		Edge(Node* source, Node* destination, int distance, int nFlights);
	};
}