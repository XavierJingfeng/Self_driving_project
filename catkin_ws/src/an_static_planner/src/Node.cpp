#include "an_static_planner/Node.h"
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <limits>

Node::Node(const int lane_num, const int sect_num): lane_num(lane_num), sect_num(sect_num){
	v = std::numeric_limits<double>::max();
	g = std::numeric_limits<double>::max();
}
Node::~Node(){}

