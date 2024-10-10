#include "edge.h"
#include "node.h"
namespace Graph
{
	class Edge
	{
	public:
		Graph::Node* source;
		Graph::Node* destination;
		int distance;
		Edge(Node* source, Node* destination, int distance)
		{
			this->source = source;
			this->destination = destination;
			this->distance = distance;
		}
	};

}