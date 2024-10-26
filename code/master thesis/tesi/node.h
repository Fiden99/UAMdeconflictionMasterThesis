#pragma once
#include <vector>
#include <map>
#include "edge.h"
#include <optional>
namespace Graph
{
	class Edge;
	//use to use reference std::reference_wrapper
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
		Edge* getInEdge(int nodeID);
		Edge* getOutEdge(int nodeID);
	};
}
