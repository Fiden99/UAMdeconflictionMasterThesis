#pragma once
#include <vector>
#include <map>
#include <optional>
#include "edge.h"
namespace Graph
{
	class Edge;
	class Node
	{
	private:
		static inline int counter{ 0 };
	public:
		int id;
		//TODO capire come inserire l'indirizzo o la referenza
		std::vector<Edge*> inArcs;
		std::vector<Edge*> outArcs;
		//angoli, con this il nodo angolato, x nel modello
		std::map<std::pair<Node*, Node*>, double> angleP;
		std::map<std::pair<Node*, Node*>, double> angleM;
		std::map<std::pair<Node*, Node*>, double> anglePM;
		//int* t_cost_ear;
		Node();
		Node(int id);
		Node(int id, std::vector<Edge*> inArcs, std::vector<Edge*> outArcs);
		Node(int id, std::vector<Edge*> inArcs, std::vector<Edge*> outArcs, std::map<std::pair<Node*, Node*>, double> angleP, std::map<std::pair<Node*, Node*>, double> angleM, std::map<std::pair<Node*, Node*>, double> anglePM);
		//utility functions
		Edge* getInEdge(const int nodeID);
		Edge* getOutEdge(const int nodeID);
		bool isGoingTo(const int nodeID);
	};
}
