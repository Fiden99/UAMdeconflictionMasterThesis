
#include <vector>
#include <map>
#include <functional>
#include "edge.h"
#include "node.h"
#include <iostream>

namespace Graph
{
	Node::Node() :
		id{ counter }
	{
		++counter;
	}
	Node::Node(int id) :
		id{ id }
	{
	}
	Node::Node(int id, std::vector<Edge*> inArcs, std::vector<Edge*> outArcs) :
		id{ id },
		inArcs{ inArcs },
		outArcs{ outArcs }
	{
	}
	Node::Node(int id, std::vector<Edge*> inArcs, std::vector<Edge*> outArcs, std::map<std::pair<Node*, Node*>, double> angleP, std::map<std::pair<Node*, Node*>, double> angleM, std::map<std::pair<Node*, Node*>, double> anglePM) :
		id{ id },
		inArcs{ inArcs },
		outArcs{ outArcs },
		angleP{ angleP },
		angleM{ angleM },
		anglePM{ anglePM }
	{
	}
	Edge* Node::getInEdge(const int nodeID)
	{
		auto it = find_if(inArcs.begin(), inArcs.end(), [nodeID](Edge* edge) {return edge->source->id == nodeID; });
		if (it == inArcs.end())
			throw std::exception("Edge not found");
		return *it;
	}
	Edge* Node::getOutEdge(const int nodeID)
	{
		auto it = find_if(outArcs.begin(), outArcs.end(), [nodeID](Edge* edge) {return edge->destination->id == nodeID; });
		if (it == outArcs.end())
			throw std::exception("Edge not found");
		return *it;
	}

	bool Node::isGoingTo(const int nodeID)
	{
		auto it = find_if(outArcs.begin(), outArcs.end(), [nodeID](Edge* edge) {return edge->destination->id == nodeID; });
		return it != outArcs.end();
	}
}