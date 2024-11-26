#pragma once
#include "graph.h"
namespace Graph {

	class Hypergraph : public Graph
	{
	public:
		std::map<Node*, std::vector<Edge*>> hyperEdges;
		Hypergraph();
		Hypergraph(Graph);
	};
}