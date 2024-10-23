#pragma once
#include <vector>
#include "node.h"
namespace Graph
{
	class Graph
	{
	public:
		std::vector<Node*> nodes;
		Graph(int);
	};
}
