
#include <string>
#include <stdexcept>
#include <fstream> // Add this line to include the ifstream header
#include <vector> // Add this line to include the vector header
#include <iostream>

#include "flights.h"
#include "node.h"
#include "edge.h"
#include "graph.h"


int findNumFlights(std::string& line) {
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

int findNumNodes(std::string& line) {
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



void reader(std::string& filename) {
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
