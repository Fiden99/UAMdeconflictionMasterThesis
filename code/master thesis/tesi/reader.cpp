
#include <string>
#include <stdexcept>
#include <fstream> // Add this line to include the ifstream header
#include <vector> // Add this line to include the vector header
#include <iostream>
#include <functional>
#include <sstream> 

#include "flights.h"
#include "node.h"
#include "edge.h"
#include "graph.h"


int findNumFlights(std::string& line)
{
	if (line.find("param nf:=") != std::string::npos) {
		// Trova la posizione del segno di uguale
		size_t pos = line.find(":=");
		if (pos != std::string::npos) {
			// Estrai il valore dopo il segno di uguale
			return std::stoi(line.substr(pos + 2));
		}
	}
	throw std::runtime_error("nf not found");
}

int findNumNodes(std::string& line)
{
	if (line.find("param nn:=") != std::string::npos) {
		// Trova la posizione del segno di uguale
		size_t pos = line.find(":=");
		if (pos != std::string::npos) {
			// Estrai il valore dopo il segno di uguale
			return std::stoi(line.substr(pos + 2));
		}
	}
	throw std::runtime_error("nn not found");
}


void readingNum(std::string& line, int& index, int& value)
{
	std::istringstream iss(line);
	if (iss >> index >> value)
		return;
	throw std::runtime_error("Error reading numbers from line");
}

void readMatrix(std::ifstream& file, Graph::Graph& graph, const int nn)
{
	std::string line;
	for (int i = 0; i < nn; ++i)
	{
		if (!getline(file, line))
			throw std::runtime_error("Error reading the file");
		std::istringstream iss(line);
		std::string readed;
		iss >> readed;
		int rowIndex{ std::stoi(readed) };
		for (int j = 0; j < nn; ++j)
		{
			iss >> readed;
			if (readed == ".")
				continue;
			if (readed == ";" || readed == ".;")
				return;
			int distance{ std::stoi(readed) };
			graph.nodes[rowIndex]->outArcs.push_back(new Graph::Edge{ graph.nodes[rowIndex], graph.nodes[j], distance });
			graph.nodes[j]->inArcs.push_back(new Graph::Edge{ graph.nodes[rowIndex], graph.nodes[j], distance });
		}
	}
}


void reader(std::string& filename)
{
	// Open the text file named "input.txt"
	std::ifstream file(filename);

	// give error if the file is not opened successfully
	if (!file.is_open()) {
		throw std::runtime_error("Error opening the file!");
	}

	std::string line;
	//reading the first line, containing the number of flights
	if (!getline(file, line))
		throw std::runtime_error("Error reading the file");
	int nf = findNumFlights(line);
	std::vector<Graph::Flight> flights(static_cast<size_t>(nf));
	std::cout << "num flights: " << nf << std::endl;
	//reading the second line, containing the number of nodes
	if (!getline(file, line))
		throw std::runtime_error("Error reading the file");
	int nn = findNumNodes(line);
	Graph::Graph graph(nn);
	std::cout << "num nodes:" << nn << std::endl;
	while (getline(file, line))
	{
		//skip E, getting line but searching for s
		if (line.find("param s:=") != std::string::npos) {
			break;
		}
	}
	//ho letto param s:=, devo leggere i valori di s
	for (int i = 0; i < nf; ++i)
	{
		int index{};
		int value{};
		if (!getline(file, line))
			throw std::runtime_error("Error reading the file");
		readingNum(line, index, value);
		flights[index].source = graph.nodes[value];
	}
	//reading end point for flights
	if (!getline(file, line))
		throw std::runtime_error("Error reading the file");
	if (line.find("param e:=") == std::string::npos)
		throw std::runtime_error("Couldn't find e");
	for (int i = 0; i < nf; ++i)
	{
		int index{};
		int value{};
		if (!getline(file, line))
			throw std::runtime_error("Error reading the file");
		readingNum(line, index, value);
		flights[index].destination = graph.nodes[value];
	}
	//only for test
	for (int i = 0; i < nf; ++i)
	{
		std::cout << "flight " << i << " source: " << flights[i].source->id << " destination: " << flights[i].destination->id << std::endl;
	}
	//reading the distance
	if (!getline(file, line))
		throw std::runtime_error("Error reading the file");
	if (line.find("param d:") == std::string::npos)
		throw std::runtime_error("Couldn't find d");
	readMatrix(file, graph, nn);
	//testing graph
	for (auto i : graph.nodes)
	{
		std::cout << "node " << i->id << " has " << i->inArcs.size() << " inArcs" << std::endl;
	}





	// skip the lines defining graph, we will use the distance
	/*for (int i = 0; i <= nn; i++) {
		if ()
			throw std::runtime_error("Error reading the file");
	}
	*/

	//remember to use move semantic to avoid copy

// Close the file
	file.close();
}
