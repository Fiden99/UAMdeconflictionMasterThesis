#pragma once
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
		//TODO capire come gestire v_min e v_max che dovrebbero essere vettori/array
		/*int v_min;
		int v_max;
		*/
		Edge();
		Edge(Node* source, Node* destination, int distance);
	};
}