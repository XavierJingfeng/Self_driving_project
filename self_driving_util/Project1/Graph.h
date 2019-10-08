#ifndef _GRAPH_H_
#define _GRAPH_H_
#include "Node.h"
#include <vector>
#include <utility>
#include <functional>

class Graph
{
public:
	Node const * start;
	Node const * goal;
	std::vector<std::vector<Node>> graph;

	Graph(const char* filename);
	Graph(const Graph& real_graph);
	~Graph();
	Node const * getStart();
	Node const * getGoal();
	void setStart(Node* node);

	int getRowSize() const;
	int getColSize() const;
	void printGraph();

};

#endif
