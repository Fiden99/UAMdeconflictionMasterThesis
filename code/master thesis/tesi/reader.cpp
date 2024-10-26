
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

enum class typeAngle
{
	angleP,
	angleM,
	anglePM
};

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

void readMatrix(std::ifstream& file, Graph::Graph& graph, const int nn, const int nf)
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
			auto edge = new Graph::Edge{ graph.nodes[rowIndex], graph.nodes[j], distance,nf };
			graph.nodes[rowIndex]->outArcs.push_back(edge);
			graph.nodes[j]->inArcs.push_back(edge);
		}
	}
}
void readSpeed(std::ifstream& file, Graph::Graph& graph, const int nn, const bool find_v_min)
{
	std::string line;
	while (getline(file, line))
	{
		std::istringstream iss(line);
		std::string read1, read2, read3, read4;
		while (iss >> read1)
		{
			if (read1 == ";")
			{
				return;
			}
			if (!(iss >> read2 >> read3 >> read4))
				throw std::runtime_error("Error reading numbers from line");
			int flight{ std::stoi(read1) };
			int n1{ std::stoi(read2) };
			int n2{ std::stoi(read3) };
			double speed{ std::stod(read4) };
			auto edge = graph.nodes[n1]->getOutEdge(n2);
			if (find_v_min)
			{
				edge->v_min[flight] = speed;
			}
			else
			{
				edge->v_max[flight] = speed;
			}
		}

	}

}
void readAngle(std::ifstream& file, Graph::Graph& graph, const typeAngle type)
{
	std::string line;
	while (getline(file, line))
	{
		std::istringstream iss(line);
		std::string read1, read2, read3, read4;
		while (iss >> read1)
		{
			if (read1 == ";")
			{
				return;
			}
			if (!(iss >> read2 >> read3 >> read4))
				throw std::runtime_error("Error reading numbers from line");
			int x{ std::stoi(read1) };
			int x1{ std::stoi(read2) };
			int x2{ std::stoi(read3) };
			double angle{ std::stod(read4) };
			//std::map<std::pair<Node*, Node*>, float> angleM;
			switch (type)
			{
			case typeAngle::angleM:
				graph.nodes[x]->angleM[std::make_pair(graph.nodes[x1], graph.nodes[x2])] = angle;
				break;
			case typeAngle::angleP:
				graph.nodes[x]->angleP[std::make_pair(graph.nodes[x1], graph.nodes[x2])] = angle;
				break;
			case typeAngle::anglePM:
				graph.nodes[x]->anglePM[std::make_pair(graph.nodes[x1], graph.nodes[x2])] = angle;
				break;
			default:
				throw std::runtime_error("Error angle");
			}
		}
	}
}



int readSafetyDistance(std::string& line)
{
	if (line.find("param D:=") != std::string::npos) {
		// Trova la posizione del segno di uguale
		size_t pos = line.find(":=");
		if (pos != std::string::npos) {
			// Estrai il valore dopo il segno di uguale
			return std::stoi(line.substr(pos + 2));
		}
	}
	throw std::runtime_error("D not found");
}

void readTime(std::ifstream& file, std::vector<Graph::Flight*>& flights, const int nn, const int nf, const bool isEar)
{
	std::string line;
	for (int i = 0; i < nf; ++i)
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
			if (readed == ";")
				return;
			int time{ std::stoi(readed) };
			if (isEar)
			{
				flights[rowIndex]->earliestHatTime[j] = time;
			}
			else
			{
				flights[rowIndex]->latestHatTime[j] = time;
			}
		}
	}
}

void reader(std::string& filename, Graph::Graph& graph, std::vector<Graph::Flight*>& flights)
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
	//std::vector<Graph::Flight*> flights(static_cast<size_t>(nf));
	flights.resize(static_cast<size_t>(nf));
	//reading the second line, containing the number of nodes
	if (!getline(file, line))
		throw std::runtime_error("Error reading the file");
	int nn = findNumNodes(line);
	//Graph::Graph graph(nn);
	graph = Graph::Graph(nn);

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
		flights[index] = new Graph::Flight{ graph.nodes[value], nullptr, nn };
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
		flights[index]->destination = graph.nodes[value];
	}
	//reading the distance
	if (!getline(file, line))
		throw std::runtime_error("Error reading the file");
	if (line.find("param d:") == std::string::npos)
		throw std::runtime_error("Couldn't find d");
	readMatrix(file, graph, nn, nf);
	if (!getline(file, line))
		throw std::runtime_error("Error reading the file");
	if (line.find("param v_min:=") == std::string::npos)
		throw std::runtime_error("Couldn't find v_min");
	//read vmin
	readSpeed(file, graph, nn, true);

	if (!getline(file, line))
		throw std::runtime_error("Error reading the file");
	if (line.find("param v_max:=") == std::string::npos)
		throw std::runtime_error("Couldn't find v_max");
	//read vmax
	readSpeed(file, graph, nn, false);

	if (!getline(file, line))
		throw std::runtime_error("Error reading the file");
	if (line.find("param angleM:=") == std::string::npos)
		throw std::runtime_error("Couldn't find angleM");
	//read angle
	readAngle(file, graph, typeAngle::angleM);
	if (!getline(file, line))
		throw std::runtime_error("Error reading the file");
	if (line.find("param angleP:=") == std::string::npos)
		throw std::runtime_error("Couldn't find angleP");
	//read angle
	readAngle(file, graph, typeAngle::angleP);
	if (!getline(file, line))
		throw std::runtime_error("Error reading the file");
	if (line.find("param anglePM:=") == std::string::npos)
		throw std::runtime_error("Couldn't find anglePM");
	//read angle
	readAngle(file, graph, typeAngle::anglePM);

	//todo angles
	if (!getline(file, line))
		throw std::runtime_error("Error reading the file");
	if (line.find("param D:=") == std::string::npos)
		throw std::runtime_error("Couldn't find D");
	//read safety distance
	Graph::Flight::safetyDistance = readSafetyDistance(line);
	//read ear time
	if (!getline(file, line))
		throw std::runtime_error("Error reading the file");
	if (line.find("param t_hat_ear:") == std::string::npos)
		throw std::runtime_error("Couldn't find t_hat_ear");
	readTime(file, flights, nn, nf, true);
	//read lat time
	if (!getline(file, line))
		throw std::runtime_error("Error reading the file");
	if (line.find("param t_hat_lat:") == std::string::npos)
		throw std::runtime_error("Couldn't find t_lat");
	readTime(file, flights, nn, nf, false);




	//remember to use move semantic to avoid copy

// Close the file
	file.close();
}
