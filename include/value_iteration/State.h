#ifndef VALUE_STATE_
#define VALUE_STATE_

#include "Action.h"

namespace value_iteration{

using namespace std;

class State{
public: 
	uint64_t _cost;
	uint64_t _penalty;
	int _ix, _iy, _it;
	bool _free;
	bool _final_state;
	Action *_optimal_action;

	State(int x, int y, int theta, const nav_msgs::OccupancyGrid &map, int margin, int x_num);
};

}

#endif
