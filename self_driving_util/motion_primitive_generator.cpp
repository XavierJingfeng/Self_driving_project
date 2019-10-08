#include <cstdlib>
#include <iostream>
#include <cmath>
#include <utility>
#include <vector>
#include <iomanip>
#include <string>
#include <cmath>

class State{
	// nodes in a graph
public:
	
	vector<double> coordinates; // (x, y, z, v, t) : x,y,z are spacial coordinate, v is the velocity(constant), t is the time
	double time;
	double velocity;
	bool expanded;
	pair<int, int> station_lane;

	pair<int, int> Key;  // first for v-value, second for f-value
	int hval;
	int gval;


};

class Motion_primitive{
	// edges in a graph
public:
	const int station_shift_;
	const int lane_shift_; // +1=left_shift, -1=right_shift, 0 = straight
	const int nb_of_inter_elem_;

	State* origin;
	int cost;
	vector<vector<double>> intermediate_pts;
	double length;
	double total_time;

	Motion_primitive(const int station_shift, const int lane_shift, const int nb_of_inter_elem) 
				: station_shift_(station_shift),lane_shift_(lane_shift), nb_of_inter_elem_(nb_of_inter_elem){

		std::vector<double> n1(start->coordinates);
		std::vector<double> n2;
		std::vector<double> n3;
		std::vector<double> n4(end->coordinates);
		
		cost =  station_shift_*std::abs(lane_shift_) + station_shift_;

		
		// create intermediate pts
		int coordinate_size = origin->coordinates.size();
		intermediate_pts.push_back(origin->coordinates);
		double s = 1./(nb_of_inter_elem_);

		if(lane_shift == 0){
			// [left, right)
			for(int i = 1; i<nb_of_inter_elem_; ++i){
				double x = STATION_LENGTH*s*i;
				std::vector<double> coordinates(coordinate_size);

				intermediate_pts.push_back(geo_point);
			}
		}else{
			
		}


		


			
	}

	State* get_successor(const State& origin){

	}

	State* get_predecessor(const State& origin){

	}



};

int main(int argc, char const *argv[])
{
	std::pair<double, double> n1(0,0);
	std::pair<double, double> n2(15,0);
	std::pair<double, double> n3(35,3.7);
	std::pair<double, double> n4(50,3.7);
	int elem_nb = 100;
	double s = 1./(elem_nb);
	std::cout.setf(std::ios::fixed);
	for(int i = 0; i<=elem_nb; i++){
		double x = (1-s*i)*(1-s*i)*(1-s*i)*n1.first + 3*(1-s*i)*(1-s*i)*s*i*n2.first + 3*(1-s*i)*s*i*s*i*n3.first + s*i*s*i*s*i*n4.first;
		double y = (1-s*i)*(1-s*i)*(1-s*i)*n1.second + 3*(1-s*i)*(1-s*i)*s*i*n2.second + 3*(1-s*i)*s*i*s*i*n3.second + s*i*s*i*s*i*n4.second;
		double x_prime = (9*n2.first - 9*n3.first + 3*n4.first - 3*n1.first)*s*i*s*i + (6*n1.first - 12*n2.first + 6*n3.first)*s*i + (3*n2.first - 3*n1.first);
		double y_prime = (9*n2.second - 9*n3.second + 3*n4.second - 3*n1.second)*s*i*s*i + (6*n1.second - 12*n2.second + 6*n3.second)*s*i + (3*n2.second - 3*n1.second);
		double x_prime_prime = 2*(9*n2.first - 9*n3.first + 3*n4.first - 3*n1.first)*s*i + (6*n1.first - 12*n2.first + 6*n3.first);
		double y_prime_prime = 2*(9*n2.second - 9*n3.second + 3*n4.second - 3*n1.second)*s*i +6*n1.second - 12*n2.second + 6*n3.second;
		std::cout<<std::setprecision(5)<<x<<","<<y<<","<<x_prime<<","<<y_prime<<","<<x_prime_prime<<","<<y_prime_prime<<std::endl;
	}
	std::cout.unsetf(std::ios::fixed);
	return 0;
}
