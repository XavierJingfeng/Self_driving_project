#include "an_static_planner/motion_primitives.h"
#include "geometry_msgs/Pose2D.h"
#include <cstdlib>
#include <iostream>
#include <cmath>
#include <utility>
#include <vector>
#include <iomanip>
#include <string>
#include <cmath>


// constexpr double pi() { return std::atan(1)*4; } 

motion_primitive::motion_primitive(unsigned id, double length, double width): id(id){
		// bezier curve
	int elem_nb = 100;
	double s = 1./elem_nb;
	if(id == 1){
		// left_shift 3.7m and forward 100m
		length = length*2;
		lane_shift = 1;
		sect_shift = 2;
		std::pair<double, double> n1(0,0);
		std::pair<double, double> n4(length, width);
		std::pair<double, double> n2(0.3*length, 0);
		std::pair<double, double> n3(0.7*length, width);
		
		for(int i = 0; i<elem_nb; i++){
			double x = (1-s*i)*(1-s*i)*(1-s*i)*n1.first + 3*(1-s*i)*(1-s*i)*s*i*n2.first + 3*(1-s*i)*s*i*s*i*n3.first + s*i*s*i*s*i*n4.first;
			double y = (1-s*i)*(1-s*i)*(1-s*i)*n1.second + 3*(1-s*i)*(1-s*i)*s*i*n2.second + 3*(1-s*i)*s*i*s*i*n3.second + s*i*s*i*s*i*n4.second;
			double x_prime = (9*n2.first - 9*n3.first + 3*n4.first - 3*n1.first)*s*i*s*i + (6*n1.first - 12*n2.first + 6*n3.first)*s*i + (3*n2.first - 3*n1.first);
			double y_prime = (9*n2.second - 9*n3.second + 3*n4.second - 3*n1.second)*s*i*s*i + (6*n1.second - 12*n2.second + 6*n3.second)*s*i + (3*n2.second - 3*n1.second);
			double theta = std::atan2(y_prime, x_prime);
			geometry_msgs::Pose2D pose;
			pose.x = x;
			pose.y = y;
			pose.theta = theta;
			inner_pts.push_back(pose);
		}

	}else if (id == 2){
		//right_shift
		lane_shift = -1;
		sect_shift = 2;
		length = length*2;
		std::pair<double, double> n1(0,0);
		std::pair<double, double> n4(length, -width);
		std::pair<double, double> n2(0.3*length, 0);
		std::pair<double, double> n3(0.7*length, -width);	

		for(int i = 0; i<elem_nb; i++){
			double x = (1-s*i)*(1-s*i)*(1-s*i)*n1.first + 3*(1-s*i)*(1-s*i)*s*i*n2.first + 3*(1-s*i)*s*i*s*i*n3.first + s*i*s*i*s*i*n4.first;
			double y = (1-s*i)*(1-s*i)*(1-s*i)*n1.second + 3*(1-s*i)*(1-s*i)*s*i*n2.second + 3*(1-s*i)*s*i*s*i*n3.second + s*i*s*i*s*i*n4.second;
			double x_prime = (9*n2.first - 9*n3.first + 3*n4.first - 3*n1.first)*s*i*s*i + (6*n1.first - 12*n2.first + 6*n3.first)*s*i + (3*n2.first - 3*n1.first);
			double y_prime = (9*n2.second - 9*n3.second + 3*n4.second - 3*n1.second)*s*i*s*i + (6*n1.second - 12*n2.second + 6*n3.second)*s*i + (3*n2.second - 3*n1.second);
			double theta = std::atan2(y_prime, x_prime);
			geometry_msgs::Pose2D pose;
			pose.x = x;
			pose.y = y;
			pose.theta = theta;
			inner_pts.push_back(pose);
		}

	}else if(id == 3){
		//straight
		lane_shift = 0;
		sect_shift = 1;
		std::pair<double, double> n1(0,0);
		std::pair<double, double> n4(length, 0);
		std::pair<double, double> n2(0.3*length, 0);
		std::pair<double, double> n3(0.7*length, 0);

		for(int i = 0; i<elem_nb; i++){
			double x = (1-s*i)*(1-s*i)*(1-s*i)*n1.first + 3*(1-s*i)*(1-s*i)*s*i*n2.first + 3*(1-s*i)*s*i*s*i*n3.first + s*i*s*i*s*i*n4.first;
			double y = (1-s*i)*(1-s*i)*(1-s*i)*n1.second + 3*(1-s*i)*(1-s*i)*s*i*n2.second + 3*(1-s*i)*s*i*s*i*n3.second + s*i*s*i*s*i*n4.second;
			double theta = 0;
			geometry_msgs::Pose2D pose;
			pose.x = x;
			pose.y = y;
			pose.theta = theta;
			inner_pts.push_back(pose);
		}
	}
}

motion_primitive::~motion_primitive(){}
