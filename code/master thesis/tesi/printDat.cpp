#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include "graph.h"
#include "flights.h"

enum class typeAngle
{
	angleP,
	angleM,
	anglePM
};

void printArcDistance(std::ofstream& file, const Graph::Graph& graph)
{
	file << "param: E: d:=\n";
	for (const auto node : graph.nodes)
	{
		for (const auto edge : node->outArcs)
		{
			file << edge->source->id << " " << edge->destination->id << " " << edge->distance << "\n";
		}
	}
	file << ";\n";
}
/*void printMatrix(std::ofstream& file, const Graph::Graph& graph, const bool isDistance)
{
	if (isDistance)
		file << "set D:";
	else
		file << "set E:";

	for (int i = 0; static_cast<size_t>(i) < graph.nodes.size(); ++i)
	{
		file << "\t" << i;
	}
	file << ":=" << std::endl;
	for (int i = 0; static_cast<size_t>(i) < graph.nodes.size(); ++i)
	{
		file << i << "\t";
		for (int j = 0; static_cast<size_t>(j) < graph.nodes.size(); ++j)
		{
			if (isDistance)
			{
				if (graph.nodes[i]->isGoingTo(j))
					file << graph.nodes[i]->getOutEdge(j)->distance << "\t";
				else
					file << ".\t";
			}
			else
			{
				if (graph.nodes[i]->isGoingTo(j))
					file << "+\t";
				else
					file << "-\t";
			}
		}
		if (i == graph.nodes.size() - 1)
			file << ";";
		file << std::endl;
	}

}*/

void printSpeed(std::ofstream& file, const Graph::Graph& graph, const int nf, const bool isMin)
{
	file << "param ";
	if (isMin)
		file << "v_min";
	else
		file << "v_max";
	file << ":=" << std::endl;
	for (int flight = 0; flight < nf; ++flight)
	{
		for (int nodeIndex = 0; static_cast<size_t>(nodeIndex) < graph.nodes.size(); nodeIndex++)
		{
			for (const auto edge : graph.nodes[nodeIndex]->outArcs)
				if (isMin)
					file << flight << " " << edge->source->id << " " << edge->destination->id << " " << edge->v_min[flight] << "\t\t";
				else
					file << flight << " " << edge->source->id << " " << edge->destination->id << " " << edge->v_max[flight] << "\t\t";
			file << std::endl;
		}
		if (flight == nf - 1)
			file << ";" << std::endl;

	}
}

void printAngle(std::ofstream& file, const Graph::Graph& graph, const typeAngle type)
{
	file << "param ";
	switch (type)
	{
	case typeAngle::angleP:
		file << "angleP";
		break;
	case typeAngle::angleM:
		file << "angleM";
		break;
	case typeAngle::anglePM:
		file << "anglePM";
		break;
	default:
		throw std::runtime_error("Invalid type");
	}
	file << ":=" << std::endl;
	for (const auto node : graph.nodes)
	{
		switch (type)
		{
		case typeAngle::angleP:
			for (const auto& angle : node->angleP)
				file << node->id << " " << angle.first.first->id << " " << angle.first.second->id << " " << angle.second << "\t\t";
			break;
		case typeAngle::angleM:
			for (const auto& angle : node->angleM)
				file << node->id << " " << angle.first.first->id << " " << angle.first.second->id << " " << angle.second << "\t\t";
			break;
		case typeAngle::anglePM:
			for (const auto& angle : node->anglePM)
				file << node->id << " " << angle.first.first->id << " " << angle.first.second->id << " " << angle.second << "\t\t";
			break;
		default:
			throw std::runtime_error("Invalid type");
		}
		file << std::endl;
	}
	file << ";" << std::endl;
}
void printTime(std::ofstream& file, const std::vector<Graph::Flight*>& flights, const bool isEar)
{
	file << "param ";
	if (isEar)
		file << "t_hat_ear";
	else
		file << "t_hat_lat";
	file << ":=" << std::endl;
	for (int i = 0; static_cast<size_t> (i) < flights.size(); ++i)
	{
		file << i << " ";
		if (isEar)
			for (auto time : flights[i]->earliestHatTime)
				file << time << " ";
		else
			for (auto time : flights[i]->latestHatTime)
				file << time << " ";
		if (i == flights.size() - 1)
			file << ";";
		file << std::endl;
	}
}

void printDat(std::string& filename, Graph::Graph& graph, std::vector<Graph::Flight*>& flights)
{
	// Crea un oggetto ofstream e apri il file
	std::ofstream file(filename);

	// Controlla se il file è stato aperto correttamente
	if (!file.is_open())
		throw std::runtime_error("Impossibile aprire il file " + filename);
	file << "param nf:=" << flights.size() << ";" << std::endl;
	file << "param nn:=" << graph.nodes.size() << ";" << std::endl;
	//print E
	printArcDistance(file, graph);
	//print s
	file << "param s:=" << std::endl;
	for (int i = 0; static_cast<size_t>(i) < flights.size(); ++i)
	{
		if (i == flights.size() - 1)
			file << i << "\t" << flights[i]->source->id << ";" << std::endl;
		else
			file << i << "\t" << flights[i]->source->id << std::endl;
	}
	//print e
	file << "param e:=" << std::endl;
	for (int i = 0; static_cast<size_t>(i) < flights.size(); ++i)
	{
		if (i == flights.size() - 1)
			file << i << "\t" << flights[i]->destination->id << ";" << std::endl;
		else
			file << i << "\t" << flights[i]->destination->id << std::endl;
	}
	//print D
	//we don't need anymore
	//printArcDistance(file, graph, true);
	//print v_min
	printSpeed(file, graph, flights.size(), true);
	//print v_max
	printSpeed(file, graph, flights.size(), false);
	//print angleM
	printAngle(file, graph, typeAngle::angleM);
	//print angleP
	printAngle(file, graph, typeAngle::angleP);
	//print anglePM
	printAngle(file, graph, typeAngle::anglePM);
	file << "D:=" << Graph::Flight::safetyDistance << ";" << std::endl;
	//print t_hat_ear
	printTime(file, flights, true);
	//print t_hat_lat
	printTime(file, flights, false);

	file.close();
	std::cout << "Scrittura su file completata con successo!" << std::endl;

}