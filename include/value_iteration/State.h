#ifndef VALUE_STATE_
#define VALUE_STATE_

#include "Action.h"

namespace value_iteration{

using namespace std;

class State{
public: 
	uint64_t total_cost_;
	uint64_t penalty_;
	int _ix, _iy, _it;
	bool _free;
	bool _final_state;
	Action *_optimal_action;

	State(int x, int y, int theta, const nav_msgs::OccupancyGrid &map,
		int margin, double margin_penalty, int x_num);
	State(int x, int y, int theta, unsigned int cost);
};

}

#endif
