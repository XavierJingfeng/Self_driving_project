#include "Graph.h"
#include <iostream>
#include <vector>
#include <string>
#include <cstdio>
#include <cstdlib>

using namespace std;

Graph::Graph(const char* filename){
	/* parsefile */
	FILE* fp = nullptr;
	int r = 0;
	int c = 0;
	char tmp;
	fp = fopen(filename, "r");
	if(fp!=nullptr){
		fscanf(fp, "%d\n", &r);
		fscanf(fp, "%d\n", &c);
		graph.reserve(r);
		for(int i = 0; i<r; ++i){
			vector<Node> v;
			graph.push_back(move(v));
			graph[i].reserve(c);
			for(int j = 0; j<c; ++j){
				fscanf(fp, "%c ", &tmp);
				Node n(i, j, tmp);
				graph[i].push_back(n);
				if(tmp == 's'){
					start = &graph[i][j];
				}else if(tmp == 'g'){
					goal = &graph[i][j];
				}
			}
		}
		fclose(fp);

	}else{
		fprintf(stderr, "Cannot open file: %s \n", filename);
		exit(1);
	}
	/* initialize heuristic and key */
	for(auto v: graph){
		for(auto u : v){
			u.calculate_manhattan(goal);
		}
	}
}

Graph::Graph(const Graph& real_graph){
	// create blank graph
	const int r = real_graph.getRowSize();
	const int c = real_graph.getColSize();
	graph.reserve(r);
	//Node const *real_start = real_graph.start;
	//Node const *real_goal = real_graph.goal;

	for(int i = 0; i<r; ++i){
		vector<Node> v;
		graph.push_back(v);
		graph[i].reserve(c);
		for(int j = 0; j<c; ++j){
			if(i == real_graph.start->x && j == real_graph.start->y){
				Node n(i, j, 's');
				graph[i].push_back(n);
				start = &graph[i].back();
			}else if(i == real_graph.goal->x && j == real_graph.goal->y){
				Node n(i, j, 'g');
				graph[i].push_back(n);
				goal = &graph[i].back();
			}else{
				Node n(i, j, '_');
				graph[i].push_back(n);
			}
		}
	}
	for(int i = 0; i<r; ++i){
		for(int j = 0; j<c; ++j){
			graph[i][j].calculate_manhattan(goal);
		}
	}
}

Graph::~Graph(){}

Node const* Graph::getStart(){
	return start;
}

Node const* Graph::getGoal(){
	return goal;
}

void Graph::setStart(Node* node){
	int x = node->x;
	int y = node->y;
	graph[x][y].flag = 's';
	graph[x][y].gval = 0;
	graph[x][y].key = inf;
	//graph[x][y].calculate_manhattan(goal);
	start = &graph[node->x][node->y];
}

int Graph::getRowSize() const{
	return graph.size();
}

int Graph::getColSize() const{
	return graph[0].size();
}

void Graph::printGraph(){
	for(int i = 0; i<graph.size(); ++i){
		for(int j = 0; j<graph[i].size(); ++j){
			cout<<graph[i][j].flag<<" ";
		}
		cout<<""<<endl;
	}
}
