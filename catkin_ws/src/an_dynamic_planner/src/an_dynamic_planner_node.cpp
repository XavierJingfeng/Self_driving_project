#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Time.h"
#include "std_msgs/Header.h"
#include "an_messages/lanes.h"
#include "an_messages/lane.h"
#include "an_messages/trajectory.h"
#include "an_messages/traj_pt.h"
#include "an_messages/obstacle.h"
#include "an_messages/obstacles.h"
#include "geometry_msgs/PoseStamped.h"
#include "an_dynamic_planner/motion_primitives.h"
#include "an_dynamic_planner/update_pq.h"
#include <unordered_set>
#include <utility>
#include <cmath>

/*In the planner of project 3, ARA* is implemented. Path is determined BEFORE the car will move*/

//Global variables
#define SECTION_SIZE 40 

constexpr double INF = std::numeric_limits<double>::max();


class Planner_class
{
public:
	Planner_class(){
		lanes_received = false;
		obstacles_received = false;
		pose_received = false;
		goal_received = false;
		new_plan = true;
	}
	~Planner_class(){

	}

	bool init(){
		//ros::NodeHandle n;
		ros::Rate r(0.5);
		ros::Subscriber lanes_sub = planner.subscribe("lanes",1, &Planner_class::lanesCallback, this);
		ros::Subscriber pose_sub = planner.subscribe("pose",1, &Planner_class::poseCallback, this);
		ros::Subscriber goal_sub = planner.subscribe("goal",1, &Planner_class::goalCallback, this);
		ros::Subscriber obstacles_sub = planner.subscribe("obstacles",1, &Planner_class::obstaclesCallback, this);
		while(!(lanes_received && goal_received && pose_received && obstacles_received)){
			ros::spinOnce();
			r.sleep();
		}
		
		generate_map();
		generate_motion_primitives();

		ROS_INFO("finish init");
		return true;
	}		
	void loop(){
		ros::Rate r(0.5);
		ros::Publisher path_pub = planner.advertise<an_messages::trajectory>("planner_trajectory", 1, true);
		while(ros::ok()){
			if(new_plan && lanes_received && goal_received && pose_received && obstacles_received){	
				ROS_WARN("[planner] Started planning");   
				new_plan = false;   
				double epsilon_start = 3.0;
				std::vector<Node*> final_path = plan(Start, Goal, epsilon_start);
				ROS_WARN("[planner] Finished planning"); 

				build_path(final_path, StartTime);  
				ROS_INFO("[planner] Finished sending path");
				path_pub.publish(final_traj);
			}

			ros::spinOnce();	
			r.sleep();
		}
	}

private:
	
	// 
	ros::NodeHandle planner;
	bool lanes_received;
	bool obstacles_received;
	bool pose_received;
	bool goal_received;
	bool new_plan;
	an_messages::trajectory final_traj;
	
	// motion primitives
	std::vector<motion_primitive> motion_primitives;
	double velocity = 27; // should be greater than 25m/s
	
	//Map members
	int lane_size;
	double lane_width;
	double lane_length;
	double section_length;
	double outer_radius;
	ros::Time *StartTime;

	Node* Start;
	Node* Goal;
	std::unordered_map<int, Node> Nodes_map; // key = lane_num + sect_num*lane_size
	int section_size = SECTION_SIZE;
	update_pq open_list;
	std::unordered_set<Node*, std::hash<Node*>> close_list;
	std::unordered_set<Node*, std::hash<Node*>> incons_list;
	std::vector<an_messages::obstacle> obstacles; // sorted by id
	/*
		Obstacles set
		Only shows whether a grid cell contains an obstacle or not. The collision check returns true when the grid cell contains at least one obstacle. 
	*/
	//std::unordered_set<int> obstacles_index_set; //
	 /*
		Obstacles look_up table
		Key: grid index = lane_num + sect_num * lane_size
		Value: vector of obstacles id 
		std::unordered_map<int , std::vector<int>> obstacles_index_map; 
	 */ 
	std::vector< std::unordered_map<int, std::pair<double, double> >> dynamic_obs_map; // t->idx->[x][y]

	// temp varibles
	std::pair<double, double> start; 
	std::pair<double, double> goal;
	
	

////////////////////////////////////////////////////////////////////////////////
/****************************Member functions**********************************/
////////////////////////////////////////////////////////////////////////////////


/****************************Planning algorithm***********************************/

	std::vector<Node*> plan(Node* start, Node* goal, double epsilon_start){
		// 
		
		double epsilon = epsilon_start;
		double epsilon_prime = epsilon;
		open_list.clear();
		Start->f = epsilon*calculate_heuristics_manhattan(Start);
		open_list.push(Start);
		std::vector<Node*> path = ARA_star(Start, Goal, epsilon);
		ROS_INFO("First path finished");

		while(epsilon_prime > 1){
			epsilon = epsilon - 0.5;
			ROS_INFO("epsilon: %f", epsilon);
			for(auto it = incons_list.begin(); it != incons_list.end(); it++){
				open_list.push(*it);
				// ROS_INFO("incons elements: %d, %d", (*it)->lane_num, (*it)->sect_num);
			}
			for(int i = 0; i<open_list.size(); i++){
				open_list[i]->f = open_list[i]->g + epsilon*calculate_heuristics_manhattan(open_list[i]);
			}	
			open_list.heapify();
			// ROS_INFO("before replanning");
			// ROS_INFO("**********************************");
			// for(int i = 0; i< open_list.size(); ++i){
			// 	ROS_INFO("leaving elements: %d, %d", open_list[i]->lane_num, open_list[i]->sect_num);
			// }
			// ROS_INFO("**********************************");
			path = ARA_star(Start, Goal, epsilon);
			printPath(path);
			epsilon_prime = calculate_epsilon_prime(epsilon);

			ROS_INFO("epsilon_prime: %f", epsilon_prime);
		}
		return path;
	}

	// ARA* algorithm
	std::vector<Node*> ARA_star(Node* start, Node* goal, double epsilon){
		
		
		close_list.clear();
		incons_list.clear();
		   // initialize queues (except for OPEN) 
		// ROS_INFO("#########");
		// ROS_INFO("ARA*");
		// ROS_INFO("#########");
		// ROS_INFO("map->Goal->g :%f", Goal->g);
		// ROS_INFO("map->open_list.top()->f :%f", open_list.top()->f);
		while (open_list.top() != nullptr && Goal->g > open_list.top()->f){
			// while goal cost is greater than best path found so far  
			Node* node = open_list.pop();
			
			// node.v = node.g; 
			close_list.insert(node);
			std::vector<Node*> SUCC = get_successors(node); 
			for (int idx=0; idx < SUCC.size(); idx++){
				// if (NotVisited(SUCC[idx]) ) SUCC[idx].v = SUCC[idx].g = INF;
				
				// ROS_INFO("node.lane_num : %d,  node.sect_num: %d", node->lane_num, node->sect_num);
				
				if (SUCC[idx]->g >= node->g + cost(node, SUCC[idx])) {
					/* CRITICAL CHANGE !!!!!! SUCC[idx]->g > node->g + cost(node, SUCC[idx]) => SUCC[idx]->g >= node->g + cost(node, SUCC[idx])*/
					// ROS_INFO("succ[%d]: %d, %d", idx, SUCC[idx]->lane_num, SUCC[idx]->sect_num);	

					SUCC[idx]->g = node->g + cost(node, SUCC[idx]); 
					// update the parent pointer and the motion primitive
					SUCC[idx]->parent = node;
					//ROS_INFO("SUCC[idx]: lane_num: %d, sect_num: %d", SUCC[idx]->lane_num, SUCC[idx]->sect_num); 
					SUCC[idx]->prev_motion_id = get_motion_primitive_id(SUCC[idx], node);
					if(close_list.find(SUCC[idx]) == close_list.end()) {
						
						SUCC[idx]->f = SUCC[idx]->g + epsilon*calculate_heuristics_manhattan(SUCC[idx]);
						// ROS_INFO("succ[%d]->g: %f, succ[%d]->f: %f", idx, SUCC[idx]->g, idx, SUCC[idx]->f);
						open_list.update(SUCC[idx], SUCC[idx]->f); 

					}else{incons_list.insert(SUCC[idx]);}
				}
			}
		}
		
		std::vector<Node*> path = generatePath();

		return path;
	}  


/**********************************Map info initialization*************************************/


	void generate_map(){
		// set start and goal
		int start_lane_num =  start.second/lane_width;
		int start_sect_num =  start.first/section_length;
		int goal_lane_num =  goal.second/lane_width;
		int goal_sect_num =  goal.first/section_length;

		set_Start(start_lane_num, start_sect_num);
		set_Goal(goal_lane_num, goal_sect_num);
		// load obstacles
		// for(int i = 0; i<obstacles.size(); i++){
		// 	int lane_num = obstacles[i].path[0].traj[0].position.y/lane_width;
		// 	int sect_num = obstacles[i].path[0].traj[0].position.x/section_length;
		// 	// int obs_pos_idx = obstacles[i].path[0].traj[0].position.x/section_length * lane_size + obstacles[i].path[0].traj[0].position.y/lane_width;
		// 	set_Obstacles(lane_num, sect_num);
		// }

	}

	void generate_motion_primitives(){

		for(int i = 0; i<3; ++i){
			motion_primitives.emplace_back(1+i, section_length, lane_width, velocity);
		}

		// ROS_INFO("size : %d, %d, %d", left_shift->inner_pts.size(), right_shift->inner_pts.size(), straight->inner_pts.size());
		// ROS_INFO("left_shift start : (%.4f, %.4f) end: (%.4f, %.4f), theta: %.4f"
		// 	, left_shift->inner_pts.front().x, left_shift->inner_pts.front().y, left_shift->inner_pts.back().x, left_shift->inner_pts.back().y, left_shift->inner_pts.back().theta);
		// ROS_INFO("right_shift start : (%.4f, %.4f) end: (%.4f, %.4f), theta: %.4f"
		// 	, right_shift->inner_pts.front().x, right_shift->inner_pts.front().y, right_shift->inner_pts.back().x, right_shift->inner_pts.back().y, right_shift->inner_pts.back().theta);
		// ROS_INFO("left_shift start : (%.4f, %.4f) end: (%.4f, %.4f), theta: %.4f"
		// 	, straight->inner_pts.front().x, straight->inner_pts.front().y, straight->inner_pts.back().x, straight->inner_pts.back().y, straight->inner_pts.back().theta);
	}

	void build_path(std::vector<Node*> path, ros::Time *StartTime){
		// TO BE FINISHED !!!

		// Step 1. Construct a trajectory using path

		// for(auto node : path){
		// 	ROS_INFO("lane_num: %d, sect_num: %d, motion_primitive_id: %d", node->lane_num, node->sect_num, node->prev_motion_id);
		// 	if (node->parent != nullptr) ROS_INFO(", parent lane_num: %d, parent sect_num: %d", node->parent->lane_num, node->parent->sect_num);
		// }
		//printPath(path);
		
		int inner_pts_size = motion_primitives[0].inner_pts.size();
		int path_size = path.size(); 
		// final_traj.traj.resize((path_size-1)*inner_pts_size); // only path_size-1 segments
		ROS_INFO("path_size: %d, inner_pts_size: %d", path_size, inner_pts_size);
		geometry_msgs::Pose2D pose;
		pose.x = path[path_size-1]->sect_num*section_length ;
		pose.y = path[path_size-1]->lane_num*lane_width;
		pose.theta = 0;
		ROS_INFO("x : %f, y: %f, theta: %f", pose.x, pose.y, pose.theta);
		an_messages::traj_pt pt;
		pt.position = pose;
		pt.header.frame_id = "ego";
		pt.header.seq = 0;
		pt.header.stamp = *StartTime;
		final_traj.traj.push_back(pt);
		
		
		for(int i = 1; i<(path_size-1)*inner_pts_size; ++i){
			Node* curr_node = path[path_size-2 - i/inner_pts_size];
			unsigned curr_node_mp_id = curr_node->prev_motion_id;

			geometry_msgs::Pose2D pose;
			
			pose.x = curr_node->parent->sect_num*section_length + motion_primitives[curr_node_mp_id-1].inner_pts[i%inner_pts_size][0];
			pose.y = curr_node->parent->lane_num*lane_width + motion_primitives[curr_node_mp_id-1].inner_pts[i%inner_pts_size][1];
			// std::cout<<"x: "<<pose.x<<","<<"y: "<<pose.y<<std::endl;
			pose.theta = motion_primitives[curr_node_mp_id-1].inner_pts[i%inner_pts_size][2];
			an_messages::traj_pt tmp;
			tmp.position = pose;
			tmp.header.frame_id = "ego";
			tmp.header.seq = 0;
			double dist = sqrt( (pose.x-final_traj.traj[i-1].position.x)*(pose.x-final_traj.traj[i-1].position.x) + (pose.y-final_traj.traj[i-1].position.y)*(pose.y-final_traj.traj[i-1].position.y));
			tmp.header.stamp = final_traj.traj[i-1].header.stamp + ros::Duration(dist/velocity);
			// ROS_INFO("time: : %d", tmp.header.stamp);
			final_traj.traj.push_back(tmp);
		}
		ROS_INFO("out of the loop");
		ROS_INFO("size of trajectory: %d", final_traj.traj.size());
		

	}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	double calculate_heuristics_manhattan(Node* node){
		if(node != nullptr){
			return abs(node->lane_num - Goal->lane_num) + abs(node->sect_num - Goal->sect_num);
		}
		return 0;
	}

	double cost(Node* n1, Node* n2){
		// related to motion primitives

		if(n1 != nullptr && n2 != nullptr){
			return abs(n1->lane_num - n2->lane_num)*3 + abs(n1->sect_num - n2->sect_num);
		}
		return 0;
	}

	std::vector<Node*> get_successors(Node* node){
		// In total, we have 3 motion primitives

		// 1. left_shift 2. right-shift 3. straight
		std::vector<Node*> SUCC;
		// ROS_INFO("lane_num : %d, sect_num: %d",node->lane_num, node->sect_num);
		for(auto mp : motion_primitives){
			// for each motion primitive collision check
			if(!collision_check(node, mp) 
				&& node->lane_num + mp.lane_shift>=0 
				&& node->lane_num + mp.lane_shift<=2){
				int idx = toIdx(node->lane_num + mp.lane_shift, node->sect_num + mp.sect_shift);
				auto got = Nodes_map.find(idx);

				if(got == Nodes_map.end()){
					// std::pair<int, Node> idx_node_pair(idx, Node(node->lane_num + dy[i], node->sect_num + dx[i]));
					Node tmp(node->lane_num + mp.lane_shift, node->sect_num + mp.sect_shift);
					tmp.arrival_time = node->arrival_time + mp.total_traverse_time;

					Nodes_map.insert(std::make_pair(idx,tmp));
					
					//Nodes_map.emplace(idx, Node(node->lane_num + mp.lane_shift, node->sect_num + mp.sect_shift));
					SUCC.push_back(&(Nodes_map.find(idx)->second));
					
				}else{
					// Node* tmp = const_cast<Node*>(&got->second);
					got->second.arrival_time = node->arrival_time + mp.total_traverse_time;
					SUCC.push_back(&(got->second));
				}	
			}
		}
		for(auto succ : SUCC){
			ROS_INFO("Node: (%d, %d), get successors: (%d, %d)\n", node->lane_num, node->sect_num, succ->lane_num, succ->sect_num);	
		}
		

		return SUCC;
	}

	void set_Goal(int lane_num, int sect_num){
		// Node n(lane_num, sect_num);
		int idx = toIdx(lane_num, sect_num);
		ROS_INFO("goal_idx : %d", idx);
		// std::pair<int, Node> idx_node_pair(idx, Node(lane_num, sect_num));
		Nodes_map.emplace(idx, Node(lane_num, sect_num));
		auto it  = Nodes_map.find(idx);
		Goal = &(it->second);
		// Goal = &(idx_node_pair.second);
		Goal->g = INF;
		Goal->v = INF;
		ROS_INFO("Goal %p", Goal);
	}
	void set_Start(int lane_num, int sect_num){
		// Node n(lane_num, sect_num);
		int idx = toIdx(lane_num, sect_num);
		ROS_INFO("start_idx : %d", idx);
		// std::pair<int, Node> idx_node_pair(idx, Node(lane_num, sect_num));
		Nodes_map.emplace(idx, Node(lane_num, sect_num));
		auto it = Nodes_map.find(idx);
		// Start = &(idx_node_pair.second);
		Start = &(it->second);
		Start->g = 0;
		Start->v = INF;
		Start->prev_motion_id = 0;
		Start->parent = nullptr;
		Start->arrival_time = 0;
	}

	// void set_Obstacles(int lane_num, int sect_num){
	// 	int obs_pos_idx = toIdx(lane_num, sect_num);
	// 	if(obstacles_index_set.find(obs_pos_idx) == obstacles_index_set.end()){
	// 		obstacles_index_set.insert(obs_pos_idx);
	// 	}

	// }

	bool collision_check(Node* node, motion_primitive& mp){

		/*	Check a motion primitive will lead to a collision or not. Return true if collision will happen. */
		/* extend the collison check*/
		/*
			for every inner points in a motion primitive inner point, we check at this given position and time whether there will be collision.

		*/
		//bool flag = false;
		for(int i = 0; i<mp.inner_pts.size(); ++i){
			
			double time = node->arrival_time + mp.inner_pts[i][3]; 
			double y = node->lane_num*lane_width  + mp.inner_pts[i][1];
			double x = node->sect_num*section_length + mp.inner_pts[i][0];
			int idx = toIdx(x, y);

			//ROS_INFO("time: %f", time);
			int t1 = (int) time/0.05; // round to 50ms
			int t2 = t1+1; 
			ROS_INFO("t1: %f, t2: %f\n", t1*0.05, t2*0.05);
			auto got1 = dynamic_obs_map[t1].find(idx);
			auto got2 = dynamic_obs_map[t2].find(idx);
			if( got1 != dynamic_obs_map[t1].end() &&  got2 != dynamic_obs_map[t2].end()){
				double x1 = got1->second.first;
				double y1 = got1->second.second;
				double x2 = got2->second.first;
				double y2 = got2->second.second;
				if(dist(x, y, x1, y1) < 4*outer_radius || dist(x, y, x2, y2) < 4*outer_radius)  return true;
			}	
			// calculate  linear interpolation 
		}
		return false;

	}

	int toIdx(int lane_num, int sect_num){
		return lane_num + sect_num*lane_size;
	}
	int toIdx(double x, double y){
		int lane_num = (int) y/lane_width;
		int sect_num = (int) x/section_length;
		return lane_num + sect_num*lane_size;
	}
	double dist(double x1, double y1, double x2, double y2){
		return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
	}

	std::vector<Node*> generatePath(){
		std::vector<Node*> path;
		Node* tmp = Goal;

		while(tmp != nullptr){
			path.push_back(tmp);
			tmp = tmp->parent;
		}
		return path;
	}

	void printPath(std::vector<Node*> path){
		ROS_INFO("entering print path");
		for(auto n : path){
			ROS_INFO("lane_num: %d, sect_num: %d prev_motion_id: %d", n->lane_num, n->sect_num, n->prev_motion_id);

		}
	}

	double getMin(std::unordered_set<Node*> incons_list){
		double min = INF;
		for(auto node : incons_list){
			min = std::min(node->f, min);
		}
		return min;
	}

	double calculate_epsilon_prime(double epsilon){
		double min = INF;
		for(auto node : incons_list){
			min = std::min(node->g + calculate_heuristics_manhattan(node), min);
		}
		if(open_list.top() != nullptr){
			min = std::min(min, open_list.top()->g + calculate_heuristics_manhattan(open_list.top()));	
		}
		min = std::min(epsilon, min);
		return min;
	}
	unsigned get_motion_primitive_id(Node* child, Node* parent){
		if(child != nullptr && parent != nullptr){
			if(child->lane_num == parent->lane_num){
				return 3;
			}else if(child->lane_num > parent->lane_num){
				return 1;
			}else if(child->lane_num < parent->lane_num){
				return 2;
			}

		}
		return 0;
	}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	/*************************Callback functions*****************************/
	//////////////////////////////////////////////////////////////////////////
	// an_messages::trajectory 
	void lanesCallback (const an_messages::lanes msg){
		ROS_INFO("lanes msg RECEIVED!");
		
		lane_size = msg.lanes.size();
		lane_width = msg.lanes[0].width;
		lane_length = msg.lanes[0].centerline[1].x - msg.lanes[0].centerline[0].x;
		ROS_INFO("lane_length: %.4f", lane_length);
		section_length = lane_length/SECTION_SIZE;

		lanes_received = true;
	}
	void goalCallback(const geometry_msgs::PoseStamped msg){
		ROS_INFO("goal msg RECEIVED!");
		ROS_INFO("goal position: %.4f, %.4f, %.4f", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);

		goal.first = msg.pose.position.x;
		goal.second = msg.pose.position.y;

		goal_received = true;
	}

	void poseCallback(const geometry_msgs::PoseStamped msg){
		ROS_INFO("pose msg RECEIVED!");
		ROS_INFO("Start position: %.4f, %.4f, %.4f", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
		start.first = msg.pose.position.x;
		start.second = msg.pose.position.y;
		// start_time.data = msg.header.stamp;
		pose_received = true;
	}
	void obstaclesCallback(const an_messages::obstacles msg){
		ROS_INFO("obstacles msg RECEIVED!");
		outer_radius = msg.obs[0].outer_radius;
		std::unordered_map<int, std::pair<double, double>> map;
		dynamic_obs_map.assign(msg.obs[0].path[0].traj.size(), map);
		StartTime = new ros::Time(msg.obs[0].path[0].traj[0].header.stamp.sec, msg.obs[0].path[0].traj[0].header.stamp.nsec);
		for(int i = 0; i<msg.obs.size(); i++){
			
			// ROS_INFO("traj_pt number %d", msg.obs[i].path.size());
			// ROS_INFO("%d th obs outer_radius: %.4f", i, msg.obs[i].outer_radius);
			int sec0 = msg.obs[i].path[0].traj[0].header.stamp.sec;

			int nsec0 = msg.obs[i].path[0].traj[0].header.stamp.nsec;
			//ROS_INFO("%dth start_time: %u, %u", i, sec0, nsec0);
			double time = 0;

			for(int j = 0; j<msg.obs[i].path[0].traj.size(); j++){
				an_messages::traj_pt obs = msg.obs[i].path[0].traj[j];
				if(obs.header.stamp.nsec < nsec0){
					
					time = obs.header.stamp.sec-sec0-1/1.0 + (1000000000-nsec0+obs.header.stamp.nsec)/1000000000;
				}else{
					
					time = obs.header.stamp.sec-sec0/1.0 + (obs.header.stamp.nsec-nsec0)/1000000000;
				}
				int idx = toIdx(obs.position.x, obs.position.y);				
				if(dynamic_obs_map[j].find(idx) == dynamic_obs_map[i].end()){
					dynamic_obs_map[j].insert(std::make_pair(idx, std::make_pair(obs.position.x, obs.position.y)));
				}
			}

		}
		obstacles_received = true;
	}

};


int main(int argc, char** argv)
{

	ros::init(argc, argv, "planner");
	ROS_DEBUG("[planner] Starting");
	Planner_class planner;
	if(planner.init()){
		ROS_INFO("entering loop");
		ROS_DEBUG("[planner] Entering loop");
		planner.loop();
	}

	
	return 0;
} 