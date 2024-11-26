#include "hypergraph.h"

namespace Graph {
	Hypergraph::Hypergraph()
	{
	}
	Hypergraph::Hypergraph(Graph graph)
	{
		for (auto node : graph.nodes)
		{
			nodes.push_back(node);
			hyperEdges[node] = std::vector<Edge*>();
		}
	}
}