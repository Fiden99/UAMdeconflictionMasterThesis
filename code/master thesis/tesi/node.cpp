
#include <vector>
#include <map>
#include <functional>
#include "edge.h"
#include "node.h"

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
	Node::Node(int id, std::vector<Edge*> inArcs, std::vector<Edge*> outArcs, std::map<std::pair<Node*, Node*>, float> angleP, std::map<std::pair<Node*, Node*>, float> angleM, std::map<std::pair<Node*, Node*>, float> anglePM) :
		id{ id },
		inArcs{ inArcs },
		outArcs{ outArcs },
		angleP{ angleP },
		angleM{ angleM },
		anglePM{ anglePM }
	{
	}
}