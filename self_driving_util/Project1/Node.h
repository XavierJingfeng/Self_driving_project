#include <limits>
#ifndef _NODE_H_
#define _NODE_H_


const int inf =  std::numeric_limits<int>::max();

class Node
{
public:
	const int x,y;
	int key;
	int gval;
	int hval;
	bool expanded;
	Node* parent;
	char flag;
	Node(const int x,const int y, char flag);
	virtual ~Node();
	bool operator<(const Node& other) const;
	bool operator>(const Node& other) const;
	Node& operator=(const Node& other);
	void expand(int &cnt_expanded);
	void generate(Node& child);
	void calculate_manhattan(Node const* goal);
};

struct compareNode{
	bool operator()(const Node* n1, const Node* n2) const{
		// greater comparator for minheap
		if(n1->key == n2->key){
			return n1->gval<n2->gval;
		}
		return n1->key>n2->key;
	}
};


#endif
