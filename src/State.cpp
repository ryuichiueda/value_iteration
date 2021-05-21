#include "value_iteration/ValueIterator.h"

namespace value_iteration{

State::State(int x, int y, int theta, const nav_msgs::OccupancyGrid &map, int margin, int x_num)
{
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
		//		penalty_ = 30 << ValueIterator::prob_base_bit_;
				penalty_ = (uint64_t)(30 * ValueIterator::prob_base_);
			}
		}
	}
}

}
