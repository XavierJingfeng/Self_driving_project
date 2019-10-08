#ifndef _MAP_H_
#define _MAP_H_ 

#include "an_messages/obstacles.h"
#include <vector>	
#include "an_static_planner/Node.h"
#include "an_static_planner/update_pq.h"
#include "geometry_msgs/PoseStamped.h"
#include <unordered_set>
// spatial part of the state space
class Map
{
public:
	Node* Start;
	Node* Goal;
	std::unordered_map<int, Node> Nodes_map; // key = lane_num + sect_num*lane_size
	const int lane_size;
	const int section_size;
	update_pq open_list;
	std::unordered_set<Node*, std::hash<Node*>> close_list;
	std::unordered_set<Node*, std::hash<Node*>> incons_list;

	// element in this set is a number that equals to lane_num + section_num*lane_size
	/*
		Obstacles set
		Only shows whether a grid cell contains an obstacle or not. The collision check returns true when the grid cell contains at least one obstacle. 
	*/
	std::unordered_set<int> obstacles_index_set; //
	 /*
		Obstacles look_up table
		Key: grid index = lane_num + sect_num * lane_size
		Value: vector of obstacles id 
	 */ 
	//std::unordered_map<int , std::vector<int>> obstacles_index_map; 


	Map(const int lane_size, const int section_size);
	~Map();
	void set_Goal(int lane_num, int sect_num);
	void set_Start(int lane_num, int sect_num);
	void set_Obstacles(int lane_num, int sect_num);
	double calculate_heuristics_manhattan(Node* node);
	double cost(Node* n1, Node* n2);
	std::vector<Node*> get_successors(Node* node); // create node and add it to Nodes
	std::vector<Node*> generatePath();
private:
	bool collision_check(int lane_num, int sect_num);
	int toIdx(int lane_num, int sect_num);


	
};

#endif