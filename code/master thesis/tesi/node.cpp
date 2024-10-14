
#include <vector>
#include "Edge.h"
#include "Node.h"

namespace Graph
{

	class Node
	{
	public:
		int id;
		std::vector<Edge> edges;
		int flights;
		int* v_max;
		int* v_min;
		float* angleP;
		float* angleM;
		float* anglePM;
		int* t_cost_ear;
		Node(int id, int flights)
		{
			this->id = id;
			this->flights = flights;
			this->v_max = new int[flights];
			this->v_min = new int[flights];
			this->angleP = new float[flights];
			this->angleM = new float[flights];
			this->anglePM = new float[flights];
			this->t_cost_ear = new int[flights];
		}
	};
}