#ifndef VALUE_STATE_
#define VALUE_STATE_

/*
#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include <vector>
#include <fstream>

#include "SweepWorkerStatus.h"
*/
#include "Action.h"
using namespace std;

class State{
public: 
	uint64_t _cost;
	int _ix, _iy, _it;
	bool _free;
	bool _final_state;
	Action *_optimal_action;

	State(int x, int y, int theta, const nav_msgs::OccupancyGrid &map, int margin, int x_num);
};


#endif
