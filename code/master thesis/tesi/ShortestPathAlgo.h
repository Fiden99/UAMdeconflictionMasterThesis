#pragma once
#include <vector>
#include "boost/heap/fibonacci_heap.hpp"
#include "boost/heap/priority_queue.hpp"
#include "graph.h"
#include "flights.h"

void getMinPathsFW(const Graph::Graph& graph, const std::vector<Graph::Flight*>& flights);
void getMinPathDijsktra(const Graph::Graph& graph, const std::vector<Graph::Flight*>& flights);
