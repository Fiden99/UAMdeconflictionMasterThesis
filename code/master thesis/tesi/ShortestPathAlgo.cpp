#include "ShortestPathAlgo.h"
#include <iostream>

//TODO testare se funziona


//outputArraay is an nxn matrix, where n is the number of nodes in the graph
void floydWarshall(const Graph::Graph& graph, int*** outputArray, int** precedence)
{
	for (int k = 1; k < graph.nodes.size(); ++k)
		for (int i = 0; i < graph.nodes.size(); ++i)
		{
			for (int j = 0; j < graph.nodes.size(); ++j)
			{
				{
					if (outputArray[k - 1][i][j] > (outputArray[k - 1][i][k] + outputArray[k - 1][k][j]) &&
						graph.nodes[k]->isGoingTo(j) && graph.nodes[i]->isGoingTo(k))
					{
						outputArray[k][i][j] = outputArray[k - 1][i][k] + outputArray[k - 1][k][j];
						precedence[i][j] = k;
					}
					else
						outputArray[k][i][j] = outputArray[k - 1][i][j];
				}
			}
		}
}
//find better name

void getMinPathsFW(const Graph::Graph& graph, const std::vector<Graph::Flight*>& flights)
{
	int n = graph.nodes.size();
	int*** matrix = new int** [n];
	for (int i = 0; i < n; ++i)
	{
		matrix[i] = new int* [n];
		for (int j = 0; j < n; ++j)
		{
			matrix[i][j] = new int[n];
		}
	}

	int** precedenceMatrix = new int* [n];
	for (int i = 0; i < n; ++i)
	{
		precedenceMatrix[i] = new int[n];
	}

	for (int i = 0; i < n; ++i)
	{
		for (int j = 0; j < n; ++j)
		{
			if (i == j)
				matrix[0][i][j] = 0;
			else if (graph.nodes[i]->isGoingTo(j))
				matrix[0][i][j] = graph.nodes[i]->getOutEdge(j)->distance;
			else
				matrix[0][i][j] = INT_MAX;
			precedenceMatrix[i][j] = i;
		}
	}

	floydWarshall(graph, matrix, precedenceMatrix);
	std::vector<std::pair<int, std::vector<int>>> path(flights.size());
	for (int index = 0; index < flights.size(); ++index)
	{
		const auto& flight = flights[index];
		path[index].first = matrix[n - 1][flight->source->id][flight->destination->id];
		int i = flight->destination->id;
		do
		{
			path[index].second.push_back(i);
			i = precedenceMatrix[i][flight->destination->id];
		} while (i == flight->source->id);
	}
	// deallocate memory
	for (int i = 0; i < n; ++i)
	{
		for (int j = 0; j < n; ++j)
		{
			delete[] matrix[i][j];
		}
		delete[] matrix[i];
	}
	delete[] matrix;

	for (int i = 0; i < n; ++i)
	{
		delete[] precedenceMatrix[i];
	}
	delete[] precedenceMatrix;

}



void dijkstraFibonacci(const Graph::Graph& graph, Graph::Flight* flight, std::vector<int>& y, std::vector<Graph::Node*>& pi)
{
	Graph::Node* source = flight->source;
	Graph::Node* destination = flight->destination;
	y[source->id] = 0;
	auto compareDijkstra = [&y](const Graph::Node* a, const Graph::Node* b) -> bool {
		return y[a->id] > y[b->id];
		};
	//TODO test with fibonacci and binomial heap
	boost::heap::fibonacci_heap<Graph::Node*, boost::heap::compare<decltype(compareDijkstra)>> FH(compareDijkstra);

	std::vector<boost::heap::fibonacci_heap<Graph::Node*, boost::heap::compare<decltype(compareDijkstra)>>::handle_type> handles(graph.nodes.size());
	//riempio FH
	for (auto node : graph.nodes) {
		handles[node->id] = FH.push(node);
	}
	//boost::heap::priority_queue<Graph::Node*, boost::heap::compare<decltype(compareDijkstra)>> PQ(compareDijkstra);
	while (!FH.empty())
	{
		auto j = FH.top();
		FH.pop();
		for (auto edge : j->outArcs)
		{
			if (y[j->id] + edge->distance < y[edge->destination->id])
			{
				y[edge->destination->id] = y[j->id] + edge->distance;
				pi[edge->destination->id] = j;
				FH.decrease(handles[edge->destination->id], edge->destination);
			}
		}
	}
}

void getMinPathDijsktra(const Graph::Graph& graph, const std::vector<Graph::Flight*>& flights)
{
	std::vector<int> y(graph.nodes.size(), INT_MAX);
	std::vector<Graph::Node*> pi(graph.nodes.size(), nullptr);
	std::vector<std::vector<Graph::Node*>> path(flights.size());
	std::vector<std::vector<int>> values(flights.size());

	for (int i = 0; static_cast<size_t>(i) < flights.size(); ++i)
	{
		dijkstraFibonacci(graph, flights[i], y, pi);
		values[i] = y;
		path[i] = pi;
	}
	//da rimuovere successivamente, vedo solo se funziona


}


//TODO capire come far decidere quale coda usare all'utente
//TODO usare la coda binomiale e binaria