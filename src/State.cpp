#include "value_iteration/ValueIterator.h"

namespace value_iteration{

State::State(int x, int y, int theta, const nav_msgs::OccupancyGrid &map, int margin, int x_num)
{
	_ix = x;
	_iy = y;
	_it = theta;
	_cost = ValueIterator::max_cost_;
	_penalty = 0;
	_final_state = false;
	_optimal_action = NULL;

	//_free = true;
	_free = (map.data[y*x_num + x] == 0);

	if(not _free){
		for(int ix=-margin+x; ix<=margin+x; ix++){
			for(int iy=-margin+y; iy<=margin+y; iy++){
				int pos = iy*x_num + ix;
				if(0 <= pos and pos < map.data.size() and map.data[iy*x_num + ix] != 0)
					_penalty = 60 << ValueIterator::prob_base_bit_;
			}
		}
	}
}

}
