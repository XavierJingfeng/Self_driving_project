#include "Car.h"
#include <queue>
#include <iostream>
#include <memory>
#include <vector>
#include <functional>

using namespace std;

Car::Car(const Graph& real_graph){
	// create a car
	int r = real_graph.getRowSize();
	int c = real_graph.getColSize();
	cnt_expanded = 0;
	graph_car = unique_ptr<Graph>(new Graph(real_graph));
	
	int x = graph_car->start->x;
	int y = graph_car->start->y;
	start_position = &(graph_car->graph[x][y]);

	//set neighbors
	if(x>0 && x<r-1){
		graph_car->graph[x-1][y].flag = real_graph.graph[x-1][y].flag;
		graph_car->graph[x+1][y].flag = real_graph.graph[x+1][y].flag;
	}else if(x == 0){
		graph_car->graph[x+1][y].flag = real_graph.graph[x+1][y].flag;
	}else{
		graph_car->graph[x-1][y].flag = real_graph.graph[x-1][y].flag;
	}

	if(y>0 && y<c-1){
		graph_car->graph[x][y-1].flag = real_graph.graph[x][y-1].flag;
		graph_car->graph[x][y+1].flag = real_graph.graph[x][y+1].flag;
	}else if(y == 0){
		graph_car->graph[x][y+1].flag = real_graph.graph[x][y+1].flag;
	}else{
		graph_car->graph[x][y-1].flag = real_graph.graph[x][y-1].flag;
	}
	
	/*for(int i = 0; i<r; ++i){
		for(int j = 0; j<c; ++j){
			cout<<""<<graph_car->graph[i][j].key<<"\t";
		}
		cout<<""<<endl;
	}*/
}
Car::~Car(){

}

//private:
bool Car::planning(const Graph& real_graph){
	constexpr int dx[] = {-1, 0, 1, 0};
	constexpr int dy[] = {0, -1, 0, 1};
	int Rows = graph_car->getRowSize();
	int Cols = graph_car->getColSize();
	priority_queue<Node*, vector<Node*>, compareNode> openlist;
	openlist.push(start_position);
	while( !(graph_car->goal->expanded||openlist.empty())){
		//cout<<"goal expanded ?"<<graph_car->goal->expanded<<endl;
		const Node* top = openlist.top();
		//cout<<"x, y: "<<top->x<<" "<<top->y<<endl;
		//cout<<"hval: "<<top->hval<<endl;
		openlist.pop();
		//path.push_back(top);
		const int x = top->x;
		const int y = top->y;
		Node& cur = graph_car->graph[x][y];
		if(cur.expanded){
			continue;
		}else{
			cur.expand(cnt_expanded);	
		}
		
		//cout<<"expanded: "<<cur.x<<""<<cur.y<<endl;
		//this->printResult();
		for(int i = 0; i<4; ++i){
			if( 
				x+dx[i]>=0  && x+dx[i]<Rows &&
				y+dy[i]>=0  && y+dy[i]<Cols
			)
			{// a valid neighbor
				Node* neighbor = &graph_car->graph[x+dx[i]][y+dy[i]];
				if(!neighbor->expanded && neighbor->key == inf && neighbor->flag != 'x'){
					// not yet generated 
					cur.generate(*neighbor);
					openlist.push(neighbor);
				}
				else if(!neighbor->expanded && neighbor->flag != 'x'){
					//already generated	
					cur.generate(*neighbor);
				}
			}
		}
		
	}
	if(!graph_car->goal->expanded){
		this->printResult();
		cout<<"No route found"<<endl;
		return false;
	}
	// build path
	Node* pathend = &(graph_car->graph[graph_car->goal->x][graph_car->goal->y]);
	while(pathend->parent!= nullptr){
		if(pathend->parent->flag != 's') pathend->parent->flag = 'o';
		path.push_back(pathend);
		pathend = pathend->parent;	
	}		
	this->printResult();

	return true;
}


const Node* Car::go(const Graph& real_graph){
	constexpr int dx[] = {1, 0, -1, 0};
	constexpr int dy[] = {0, 1, 0, -1};
	int Rows = graph_car->getRowSize();
	int Cols = graph_car->getColSize();
	for(int i = path.size()-1; i>=0; --i){
		Node* n = path[i];
		const int x = n->x;
		const int y = n->y;
		if(real_graph.graph[x][y].flag == 'x'){
			start_position->flag = '_';
			start_position = &(*n->parent);
			graph_car->setStart(start_position);
			cout<<"Found obstacle en-route, start replanning..."<<endl;
			return n->parent; 
		}
		for(int i = 0; i<4; ++i){
			if( 
				x+dx[i]>=0  && x+dx[i]<Rows &&
				y+dy[i]>=0  && y+dy[i]<Cols
			)
			{// a valid neighbor
				Node* neighbor = &graph_car->graph[x+dx[i]][y+dy[i]];
				if( real_graph.graph[x+dx[i]][y+dy[i]].flag == 'x'){
					neighbor->flag = real_graph.graph[x+dx[i]][y+dy[i]].flag;	
				}	
			}
		}
		// flush the footprint expanded
	}
	return path.front();
}

void Car::flushGraphAndPath(){
	int Rows = graph_car->getRowSize();
	int Cols = graph_car->getColSize();
	for(int i = 0; i< Rows; ++i){
		for(int j = 0; j<Cols; ++j){
			Node& tmp = graph_car->graph[i][j];
			tmp.parent = nullptr;
			tmp.expanded = false;
			if(tmp.flag != 's'){
				tmp.key = inf;
				tmp.gval = inf;
			}
			if(tmp.flag == 'o'){
				tmp.flag = '_';	
			}
		}
	}
	path.clear();
}

void Car::recalculate_heuristics(){
	int Rows = graph_car->getRowSize();
	int Cols = graph_car->getColSize();
	for(int i = 0; i<Rows; ++i){
		for(int j = 0; j<Cols; ++j){
			if(graph_car->graph[i][j].expanded){
				graph_car->graph[i][j].hval = graph_car->goal->gval - graph_car->graph[i][j].gval;
			}
		}
	}
}
void Car::printResult(){
	graph_car->printGraph();
}
