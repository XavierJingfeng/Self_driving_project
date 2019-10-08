#include <iostream>
#include <vector>
#include <unordered_map>
#include <algorithm>

class Node
{
public:
	int id;
	double val;
	std::vector<Node*> parents;
	std::vector<Node*> childs;
	
	Node();
	~Node();
	
};
void value_iteration(std::vector<Node*> graph)