#include "Car.h"
#include <iostream>
#include <cstring>

using namespace std;

void A_star_search(Car* car, const Graph& real_graph, bool AA_Star_On){
	cout<<"Using A_Star planning..."<<endl;

	const Node* reached = car->start_position;
	bool planning_done = false;
	int plan_cnt = 1;
	while(reached->flag != 'g'){
		if(plan_cnt == 1){
			cout<<"# of plan: "<<plan_cnt<<" (initial)"<<endl;
		}else{
			cout<<"# of plan: "<<plan_cnt<<" (replan)"<<endl;
		}
		planning_done = car->planning(real_graph);
		if(!planning_done){
			break;
		}
		reached = car->go(real_graph);
		if(AA_Star_On){
			car->recalculate_heuristics();	
		}
		car->flushGraphAndPath();
		++plan_cnt;
	}

	cout<<"Total_expanded_nodes = "<<car->cnt_expanded<<endl;
	//cout<<plan_cnt<<","<<car->cnt_expanded<<endl;
}

int main(int argc, char const *argv[])
{
	if(argc<=1 || strcmp(argv[1], "-i") != 0){
		fprintf(stderr, "Invalid argument\n");
		exit(1);
	}
	const Graph real_graph(argv[2]);
	bool AA_Star_On = false;
	if(argc >= 4 && strcmp(argv[3], "-A") == 0){
		AA_Star_On = true;
	}
	Car* car = new Car(real_graph);
	A_star_search(car, real_graph, AA_Star_On);
	return 0;
}