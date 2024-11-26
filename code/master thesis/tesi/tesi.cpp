// tesi.cpp : Questo file contiene la funzione 'main', in cui inizia e termina l'esecuzione del programma.
//

#include <iostream>
#include "ShortestPathAlgo.h"

//presente in reader.cpp
void reader(std::string& filename, Graph::Graph& graph, std::vector<Graph::Flight*>& flights);
void printDat(std::string& filename, Graph::Graph& graph, std::vector<Graph::Flight*>& flights);


int main(int argc, char* argv[])
{
	std::string filename = "C:\\Users\\Filippo\\Desktop\\materiale universitario\\magistrale\\tesi\\modelli\\metroplexTestCPP.dat";
	Graph::Graph graph;
	std::vector<Graph::Flight*> flights;
	reader(filename, graph, flights);
	getMinPathDijsktra(graph, flights);
	getMinPathsFW(graph, flights);
	std::cout << "ho letto il file" << std::endl;
	filename = "C:\\Users\\Filippo\\Desktop\\materiale universitario\\magistrale\\tesi\\modelli\\testCPP\\testGrid0.dat";
	printDat(filename, graph, flights);
	system(("notepad " + filename).c_str());
	return 0;
}

// Per eseguire il programma: CTRL+F5 oppure Debug > Avvia senza eseguire debug
// Per eseguire il debug del programma: F5 oppure Debug > Avvia debug

// Suggerimenti per iniziare: 
//   1. Usare la finestra Esplora soluzioni per aggiungere/gestire i file
//   2. Usare la finestra Team Explorer per connettersi al controllo del codice sorgente
//   3. Usare la finestra di output per visualizzare l'output di compilazione e altri messaggi
//   4. Usare la finestra Elenco errori per visualizzare gli errori
//   5. Passare a Progetto > Aggiungi nuovo elemento per creare nuovi file di codice oppure a Progetto > Aggiungi elemento esistente per aggiungere file di codice esistenti al progetto
//   6. Per aprire di nuovo questo progetto in futuro, passare a File > Apri > Progetto e selezionare il file con estensione sln
