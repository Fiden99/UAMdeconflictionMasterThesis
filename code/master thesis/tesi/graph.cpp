
#include <iostream> 
#include "graph.h"
#include "node.h"
#include <vector>
namespace Graph
{
	Graph::Graph(int n_nodes) :
		nodes(static_cast<size_t>(n_nodes))
	{
		for (int i = 0; static_cast<size_t> (i) < n_nodes; ++i)
		{
			Node* x = new Node{ i };
			nodes[i] = x;
		}
	}
}


