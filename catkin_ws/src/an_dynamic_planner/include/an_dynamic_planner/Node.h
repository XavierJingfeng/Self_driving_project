#ifndef _NODE_H_
#define _NODE_H_ 

#include "an_messages/obstacle.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

class motion_primitive;

class Node
{
public:
	const int lane_num;
	const int sect_num;

	double arrival_time;
	
	double v;
	double g;
	double f;

	Node* parent;
	int prev_motion_id;
	
	// std::vector<an_messages::obstacle*> obstacles; //contains a container of obstacles pointers, for collision check

	

	Node(const int lane_num, const int sect_num);
	~Node();
	
	
};


// bool CompareNode (const Node* n1, const Node* n2){
// 	// greater comparator for min heap
// 	return n1->v_value > n2->v_value;
// };
 	
#endif