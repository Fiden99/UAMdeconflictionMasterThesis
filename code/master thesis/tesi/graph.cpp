
#include <iostream> 
#include "graph.h"
#include "node.h"
#include <vector>
namespace Graph
{
	Graph::Graph(int n_nodes) :
		//TODO capire come gestire vettori std::array<Node> nodes(n_nodes)
		nodes(static_cast<size_t>(n_nodes))
	{
	}
}


