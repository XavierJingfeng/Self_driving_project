#include <utility>
#include <unordered_map>
#include <iostream>
#include <cstdio>

class node{

public:
	int key;
	node(int key):key(key){}
	~node(){}
};

int main(int argc, char** argv){

	std::pair<int, node> p(0, node(8));
	std::cout<<p.first<<","<<p.second.key<<std::endl;

	return 0;
}
