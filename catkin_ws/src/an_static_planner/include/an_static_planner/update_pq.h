#ifndef _UPDATE_PQ_H_
#define _UPDATE_PQ_H_ 

#include <unordered_map>
#include <vector>
#include <utility>
#include "an_static_planner/Node.h"

class update_pq
{
private:
	std::vector<Node*> pq;
	std::unordered_map<Node*, int, std::hash<Node*>> idx_map; // provide a hash function
	int percolate_up(int idx, Node* n);
	int percolate_down(int idx, Node* n);

public:
	Node* operator[](int idx);
	update_pq();
	~update_pq();
	int size();	
	void push(Node* n);
	Node* top();
	Node* pop();
	void update(Node* node, int newkey);
	void clear();
	void heapify();
	void printHeap();


};
#endif
