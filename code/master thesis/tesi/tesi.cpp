// tesi.cpp : Questo file contiene la funzione 'main', in cui inizia e termina l'esecuzione del programma.
//

#include <iostream>



#include <iostream>
#include <cstdlib>
#include <string>
#include <string_view>
#include <cmath>
//#include "Graph.h"


int getInteger(std::string_view); // forward declaration for function getInteger

int main()
{
	std::string_view name{ "Alex" };
	std::cout << "Hello World!" << '\n';
	std::cout << getInteger(name) << "," << name << ",Hello World!" << '\n';
	std::cout << std::pow(2, 3);
	std::cout << std::asin(1.0) * 2;

	return 0;
}
int getInteger(std::string_view name)
{
	std::cout << name << "\n";
	name = "Ciaone";
	std::cout << name << "\n";
	std::cout << "Enter an integer: ";
	int x;
	std::cin >> x;
	return x;
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
