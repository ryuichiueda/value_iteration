#ifndef VALUE_STATE_
#define VALUE_STATE_

#include "Action.h"

namespace value_iteration{

using namespace std;

class State{
public: 
	vector<uint64_t> total_cost_;
	vector<uint64_t> penalty_;
	int ix_, iy_, it_;
	bool free_;
	bool final_state_;

	uint64_t local_penalty_;

	vector<Action *> optimal_action_;

	State(int x, int y, int theta, const nav_msgs::OccupancyGrid &map,
		int margin, double margin_penalty, int x_num);
	State(int x, int y, int theta, unsigned int cost);

	const static unsigned char sigma_num_;
};

const unsigned char State::sigma_num_ = 4;

}


#endif
