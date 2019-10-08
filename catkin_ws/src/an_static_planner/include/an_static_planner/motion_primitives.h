#ifndef _MOTION_PRIMITIVES_H_
#define _MOTION_PRIMITIVES_H_ 

#include "an_static_planner/Node.h"
#include "an_messages/traj_pt.h"
#include "geometry_msgs/Pose2D.h"
#include <vector> 

class motion_primitive
{

public:
	/*
		Motion primitives type is uniquely defined by its id.
		id == 1 : left_shift, lane--, sect += 2;
		id == 2 : right_shift, lane++, sect += 2;
		id == 3 : straight, sect++;
	*/
	const unsigned id; // 1. left_shift 2.right_shift 3.straight
	int lane_shift;
	int sect_shift;
	std::vector<geometry_msgs::Pose2D> inner_pts;
	motion_primitive(unsigned id, double length, double width); // 1. use bezier curve
	~motion_primitive();
	
};


#endif
