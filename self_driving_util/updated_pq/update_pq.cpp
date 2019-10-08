#include <algorithm>
#include <vector>
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <unordered_map>
#include <utility>

class Node
{
public:
	int key;
	Node(int key): key(key) {
	}
	~Node(){}
		
};

bool CompareNode (const Node* n1, const Node* n2){
	// greater comparator for min heap
	return n1->key > n2->key;
}

namespace std{
	template<> struct hash<Node> {
		std::size_t operator()(const Node& n1) const{
			return (std::hash<int>()(n1.key));
		}
	}; 
}


class update_pq
{
private: 
	std::vector<Node*> pq;
	//std::unordered_map<Node*, int, std::hash<Node*>> idx_map; // provide a hash function

	std::unordered_map<Node*, int> idx_map; // provide a hash function
	int percolate_up(int idx, Node* n){
		while(idx>0){
			int swapCandidate = (idx-1)/2;
			if( CompareNode( pq[swapCandidate], pq[idx]) ){
				Node* tmp = pq[swapCandidate];
				auto candidate_pair = idx_map.find(tmp);

				pq[swapCandidate] = pq[idx];
				pq[idx] = tmp;
				if(candidate_pair != idx_map.end()){
					candidate_pair->second = idx;
				}
			}else{
				break;
			}
			idx = swapCandidate;
		}
		return idx;
	}

	int percolate_down(int idx, Node* n){
		while(2*idx+1 < pq.size()){
			int swapCandidate = 2*idx+1;
			if( swapCandidate < pq.size()-1 && CompareNode(pq[swapCandidate], pq[swapCandidate+1])){
				swapCandidate+=1;
			}
			if( CompareNode( pq[idx], pq[swapCandidate]) ){
				Node* tmp = pq[swapCandidate];
				auto candidate_pair = idx_map.find(tmp);

				pq[swapCandidate] = pq[idx];
				pq[idx] = tmp;
				if(candidate_pair != idx_map.end()){
					candidate_pair->second = idx;
				}
			}else{
				break;
			}
			
			idx = swapCandidate;
		}
		return idx;
	}


public:
	int size() const{
		return pq.size();
	}
	update_pq(){
	}
	~update_pq(){}
	void push(Node* node){
		pq.push_back(node);
		int idx = percolate_up(pq.size()-1, node);
	
		std::pair<Node*, int> p = std::make_pair(node, idx);
		idx_map.insert(p);
	}
	Node* top(){
		if(pq.empty()) return nullptr;
		return pq.front();
	}
	Node* pop(){
		if(pq.empty()) return nullptr;
		Node* top = pq.front();
		pq[0] = pq[pq.size()-1];
		pq[pq.size()-1] = top;
		pq.pop_back();
		// percolate down and update the index map
		int idx = percolate_down(0, top);
		auto pair = idx_map.find(top);
		if(pair != idx_map.end()){
			pair->second = idx;
		}
		return top;
	}
	void update(Node* node, int newkey){
		// Assuming that node is already in the pq
		auto pair = idx_map.find(node);
		int idx;
		if(pair != idx_map.end()){
			idx = pair->second; 
		}else{
			return ;
		}
		// update the key
		int oldkey = node->key;
		int new_idx;
		if(newkey > oldkey){
			node->key = newkey;
			new_idx = percolate_down(idx, node);
		}else if(newkey < oldkey){
			node->key = newkey;
			new_idx = percolate_up(idx, node);
		}else{
			return ; 
		}
		// update the index
		pair->second = new_idx;

	}
	void printHeap(){
		std::cout<<"print heap"<<std::endl;
		for(unsigned i = 0; i<pq.size(); ++i){
			std::cout<<pq[i]->key<<std::endl;
		}
	}

};


int main(int argc, char const *argv[])
{
	Node* n1 = new Node(1);
	Node* n2 = new Node(2);
	Node* n3 = new Node(3);
	Node* n4 = new Node(4);
	Node* n5 = new Node(5);
	Node* n6 = new Node(6);

	update_pq pq;
	pq.push(n5);
	pq.push(n2);	
	pq.push(n4);
	pq.printHeap();
	
	pq.push(n1);
	pq.push(n3);
	pq.push(n6);
	pq.printHeap();
	pq.update(n2, 22);
	pq.printHeap();
	pq.update(n4, 0);
	pq.printHeap();
	pq.pop();
	pq.pop();
	pq.pop();
	pq.printHeap();

	return 0;
}
