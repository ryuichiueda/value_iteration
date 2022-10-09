#include "value_iteration/ValueIterator.h"

namespace value_iteration{

State::State(int x, int y, int theta, const nav_msgs::OccupancyGrid &map,
		int margin, double margin_penalty, int x_num)
{
	if(margin_penalty > 1.0e+10)
		ROS_ERROR("TOO LARGE PENALTY TO VIOLATION OF SAFETY RADIUS");

	/*
	sigma_thresholds_.push_back(0.1);
	sigma_thresholds_.push_back(0.2);
	sigma_thresholds_.push_back(0.4);
	sigma_thresholds_.push_back(0.8);
	*/

	ix_ = x;
	iy_ = y;
	it_ = theta;
	for(int s=0; s<State::sigma_num_;s++){
		total_cost_.push_back(ValueIterator::max_cost_);
		penalty_.push_back( ValueIterator::prob_base_ );
		optimal_action_.push_back(NULL);
	}
	local_penalty_ = 0;
	final_state_ = false;
	//optimal_action_ = NULL;

	free_ = (map.data[y*x_num + x] == 0);
	if(not free_)
		return;

	for(int ix=-margin+x; ix<=margin+x; ix++){
		for(int iy=-margin+y; iy<=margin+y; iy++){
			int pos = iy*x_num + ix;
			if(0 <= pos and pos < map.data.size() and map.data[iy*x_num + ix] != 0)
				penalty_[0] = (uint64_t)(margin_penalty * ValueIterator::prob_base_) + ValueIterator::prob_base_;
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
	free_ = (cost != 255);
	for(int s=0; s<State::sigma_num_;s++){
		penalty_[s] = free_ ? (cost << ValueIterator::prob_base_bit_) : 0;
		optimal_action_.push_back(NULL);
	}
}

}
