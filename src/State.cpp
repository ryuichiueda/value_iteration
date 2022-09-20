#include "value_iteration/ValueIterator.h"

namespace value_iteration{

State::State(int x, int y, int theta, const nav_msgs::OccupancyGrid &map,
		int margin, double margin_penalty, int x_num)
{
	if(margin_penalty > 1.0e+10)
		ROS_ERROR("TOO LARGE PENALTY TO VIOLATION OF SAFETY RADIUS");

	ix_ = x;
	iy_ = y;
	it_ = theta;
	total_cost_.push_back(ValueIterator::max_cost_);
	penalty_ = ValueIterator::prob_base_;
	local_penalty_ = 0;
	final_state_ = false;
	optimal_action_ = NULL;

	free_ = (map.data[y*x_num + x] == 0);
	if(not free_)
		return;

	for(int ix=-margin+x; ix<=margin+x; ix++){
		for(int iy=-margin+y; iy<=margin+y; iy++){
			int pos = iy*x_num + ix;
			if(0 <= pos and pos < map.data.size() and map.data[iy*x_num + ix] != 0)
				penalty_ = (uint64_t)(margin_penalty * ValueIterator::prob_base_) + ValueIterator::prob_base_;
		}
	}
}

State::State(int x, int y, int theta, unsigned int cost)
{
	ix_ = x;
	iy_ = y;
	it_ = theta;
	total_cost_.push_back(ValueIterator::max_cost_);
	final_state_ = false;
	optimal_action_ = NULL;
	free_ = (cost != 255);
	penalty_ = free_ ? (cost << ValueIterator::prob_base_bit_) : 0;
}

}
