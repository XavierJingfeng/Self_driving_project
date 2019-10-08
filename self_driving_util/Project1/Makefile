Astar: Node.o Graph.o Car.o Astar.o
	g++ -o Astar -g -Wall -std=c++11 Node.o Graph.o Car.o Astar.o
Astar.o: Car.h Astar.cpp
	g++ -c -g -Wall -std=c++11 Astar.cpp
Car.o: Car.h Car.cpp
	g++ -c -g -Wall -std=c++11 Car.cpp
Graph.o: Graph.h Graph.cpp
	g++ -c -g -Wall -std=c++11 Graph.cpp
Node.o: Node.h Node.cpp
	g++ -c -g -Wall -std=c++11 Node.cpp
clear:
	rm -f *.o Astar
	
