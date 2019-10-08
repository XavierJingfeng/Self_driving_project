#include "an_static_planner/Map.h"
#include <cmath>
#include <limits>
#include "ros/ros.h"

constexpr double INF = std::numeric_limits<double>::max();

Map::Map(const int lane_size, const int section_size): lane_size(lane_size), section_size(section_size){

}
Map::~Map(){}


double Map::calculate_heuristics_manhattan(Node* node){
	if(node != nullptr){
		return abs(node->lane_num - Goal->lane_num) + abs(node->sect_num - Goal->sect_num);
	}
	return 0;
}

double Map::cost(Node* n1, Node* n2){
	if(n1 != nullptr && n2 != nullptr){
		return abs(n1->lane_num - n2->lane_num)*3 + abs(n1->sect_num - n1->sect_num);
	}
	return 0;
}

std::vector<Node*> Map::get_successors(Node* node){
	// In total, we have 3 motion primitives
	constexpr int dx[] = {2,  2, 1};
	constexpr int dy[] = {1, -1, 0};
	// 1. left_shift 2. right-shift 3. straight
	std::vector<Node*> SUCC;
	for(int i = 0; i<3; i++){
		if( node->sect_num + dx[i] < section_size 
			&& node->lane_num + dy[i] < lane_size && node->lane_num + dy[i] >=0
			&& collision_check(node->lane_num + dy[i], node->sect_num + dx[i]))
		{
			
			int idx = toIdx(node->lane_num + dy[i], node->sect_num + dx[i]); 
			std::unordered_map<int, Node>::const_iterator got = Nodes_map.find(idx);

			if(got == Nodes_map.end()){
				std::pair<int, Node> idx_node_pair(idx, Node(node->lane_num + dy[i], node->sect_num + dx[i]));
				Nodes_map.insert(idx_node_pair);
				SUCC.push_back(&idx_node_pair.second);
			}else{
				Node* tmp = const_cast<Node*>(&(*got).second);
				SUCC.push_back(tmp);
			}
		}
	}
	return SUCC;
}

void Map::set_Goal(int lane_num, int sect_num){
	// Node n(lane_num, sect_num);
	int idx = toIdx(lane_num, sect_num);
	ROS_INFO("goal_idx : %d", idx);
	std::pair<int, Node> idx_node_pair(idx, Node(lane_num, sect_num));
	Nodes_map.insert(idx_node_pair);
	Goal = &(idx_node_pair.second);
	Goal->g = -1;
	Goal->v = -1;
	ROS_INFO("Goal %p", Goal);
}
void Map::set_Start(int lane_num, int sect_num){
	// Node n(lane_num, sect_num);
	int idx = toIdx(lane_num, sect_num);
	ROS_INFO("start_idx : %d", idx);
	std::pair<int, Node> idx_node_pair(idx, Node(lane_num, sect_num));
	Nodes_map.insert(idx_node_pair);
	Start = &(idx_node_pair.second);
	Start->g = 0;
	Start->v = -1;
}

void Map::set_Obstacles(int lane_num, int sect_num){
	int obs_pos_idx = toIdx(lane_num, sect_num);
	if(obstacles_index_set.find(obs_pos_idx) == obstacles_index_set.end()){
		obstacles_index_set.insert(obs_pos_idx);
	}

}

bool Map::collision_check(int lane_num, int sect_num){

	/*	Return true if collision will happen if accessing this node. */
	/*
		1. If we use obs_index_set, we just check if this node contains obstcles or not

	*/
	int idx = toIdx(lane_num, sect_num);
	if(obstacles_index_set.find(idx) == obstacles_index_set.end()){
		return false;
	}else{
		return true;
	}
	/*
		2. If we use obs_index_map, we need to iterate over all the obstales contained or partially contained in this node 
			We don't have this difficult issue yet.
	*/ 
}

int Map::toIdx(int lane_num, int sect_num){
	return lane_num + sect_num*lane_size;
}

std::vector<Node*> Map::generatePath(){
	std::vector<Node*> path;
	Node* tmp = Goal;

	while(tmp->parent != nullptr){
		path.push_back(tmp);
		tmp = tmp->parent;
	}
	return path;
}