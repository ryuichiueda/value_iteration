#include "value_iteration/ValueIterator.h"

namespace value_iteration{

State::State(int x, int y, int theta, const nav_msgs::OccupancyGrid &map,
		int margin, double margin_penalty, int x_num)
{
	if(margin_penalty > 1.0e+10)
		ROS_ERROR("TOO LARGE PENALTY TO VIOLATION OF SAFETY RADIUS");

	_ix = x;
	_iy = y;
	_it = theta;
	total_cost_ = ValueIterator::max_cost_;
	penalty_ = 0;
	_final_state = false;
	_optimal_action = NULL;

	_free = (map.data[y*x_num + x] == 0);
	if(not _free)
		return;

	for(int ix=-margin+x; ix<=margin+x; ix++){
		for(int iy=-margin+y; iy<=margin+y; iy++){
			int pos = iy*x_num + ix;
			if(0 <= pos and pos < map.data.size() and map.data[iy*x_num + ix] != 0){
				penalty_ = (uint64_t)(margin_penalty * ValueIterator::prob_base_);
			}
		}
	}
}

State::State(int x, int y, int theta, unsigned int cost)
{
	_ix = x;
	_iy = y;
	_it = theta;
	total_cost_ = ValueIterator::max_cost_;
	_final_state = false;
	_optimal_action = NULL;
	_free = (cost != 255);
	penalty_ = _free ? cost : 0;
}

}
