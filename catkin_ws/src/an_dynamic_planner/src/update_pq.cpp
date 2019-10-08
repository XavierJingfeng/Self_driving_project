#include "an_dynamic_planner/update_pq.h"
#include <algorithm>
#include <vector>
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <unordered_map>
#include <utility>


bool CompareNode_greater (const Node* n1, const Node* n2){
	// greater comparator for min heap
 	return n1->f > n2->f;
}

// namespace std{
// 	template<> struct hash<Node> {
// 		std::size_t operator()(const Node& n1) const{
// 			return (std::hash<int>()(n1.f));
// 		}
// 	}; 
// }



int update_pq::percolate_up(int idx, Node* n){
	while(idx>0){
		int swapCandidate = (idx-1)/2;
		if( CompareNode_greater( pq[swapCandidate], pq[idx]) ){
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

int update_pq::percolate_down(int idx, Node* n){
	while(2*idx+1 < pq.size()){
		int swapCandidate = 2*idx+1;
		if( swapCandidate < pq.size()-1 && CompareNode_greater(pq[swapCandidate], pq[swapCandidate+1])){
			swapCandidate+=1;
		}
		if( CompareNode_greater( pq[idx], pq[swapCandidate]) ){
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



int update_pq::size(){
	return pq.size();
}
update_pq::update_pq(){}

update_pq::~update_pq(){}
void update_pq::push(Node* node){
	pq.push_back(node);
	int idx = percolate_up(pq.size()-1, node);

	// std::pair<Node*, int> p = std::make_pair(node, idx);
	idx_map.emplace(node, idx);
}
Node* update_pq::top(){
	if(pq.empty()) return nullptr;
	return pq.front();
}
Node* update_pq::pop(){
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
void update_pq::update(Node* node, int newv){
	// Assuming that node is already in the pq
	auto pair = idx_map.find(node);
	int idx;
	if(pair != idx_map.end()){
		idx = pair->second; 
	}else{
		push(node);
		return ;
	}
	// update the v
	int oldv = node->v;
	int new_idx;
	if(newv > oldv){
		node->v = newv;
		new_idx = percolate_down(idx, node);
	}else if(newv < oldv){
		node->v = newv;
		new_idx = percolate_up(idx, node);
	}else{
		return ; 
	}
	// update the index
	pair->second = new_idx;

}
Node* update_pq::operator[](int idx){
	return pq[idx];
}

void update_pq::heapify(){
	// for each node that has at least one child, we perform percolate down
	int size = pq.size();
	for(int i = size/2 - 1; i>=0; i--){
		Node* node = pq[i];
		
		auto pair = idx_map.find(node);
		int new_idx = percolate_down(i, pq[i]);
		// update its idx_map
		if(pair != idx_map.end()){
			pair->second = new_idx;
		}
	}
}

void update_pq::printHeap(){
	std::cout<<"print heap"<<std::endl;
	for(unsigned i = 0; i<pq.size(); ++i){
		std::cout<<pq[i]->v<<std::endl;
	}
}

void update_pq::clear(){
	pq.clear();
	idx_map.clear();
}
