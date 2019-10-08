#ifndef _CAR_H_
#define _CAR_H_
#include "Node.h"
#include "Graph.h"
#include <functional>
#include <memory>
#include <vector>


class Car
{
public:
	Node* start_position;
	std::unique_ptr<Graph> graph_car;
	int cnt_expanded;
	std::vector<Node*> path;
	Car(const Graph& real_graph);
	~Car();
	bool planning(const Graph& real_graph);
	const Node* go(const Graph& real_graph);
	void flushGraphAndPath();
	void recalculate_heuristics();
	void printResult();

};

#endif