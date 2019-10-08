#include "Node.h"
#include <limits>
#include <cmath>
#include <algorithm>

using namespace std;


Node::Node(const int x,const int y, char flag):x(x), y(y), flag(flag){
	if(flag == 's'){
		this->gval = 0;
		this->hval = inf;
	}else if(flag == 'g'){
		this->gval = inf;
		this->hval = 0;	
	}else{
		this->gval = inf;
		this->hval = inf;
	}
	this->key = inf;
	this->parent = nullptr;
	this->expanded = false;

}

Node::~Node(){}

bool Node::operator<(const Node& other) const{
	return (this->key<other.key);
}

bool Node::operator>(const Node& other) const{
	return (this->key>other.key);
}

Node& Node::operator=(const Node& other){
	this->key = other.key;	
	this->gval = other.gval;
	this->hval = other.hval;
	this->expanded = other.expanded;
	this->parent = &(*other.parent);
	this->flag = other.flag;
	return *this;
}

void Node::expand(int &cnt_expanded){
	key = gval + hval;
	expanded = true;
	++cnt_expanded;
	//flag = 'o';	
}

void Node::generate(Node& child){
	int tmp = this->gval+1;
	if(child.gval>=tmp){
		child.parent = &(*this);
		child.gval = this->gval+1;
		child.key = child.gval + child.hval;
	}		
}

void Node::calculate_manhattan(Node const* goal){
	this->hval = abs(goal->x-this->x)+abs(goal->y-this->y);
}
