#include "ValueIterator.h"
using namespace std;

State::State(int x, int y, int theta, const nav_msgs::OccupancyGrid &map, int margin, int x_num)
{
	_ix = x;
	_iy = y;
	_it = theta;
	_cost = ValueIterator::_max_cost;
	_final_state = false;
	_optimal_action = NULL;

	_free = true;
	for(int ix=-margin+x; ix<=margin+x; ix++)
		for(int iy=-margin+y; iy<=margin+y; iy++){
			int pos = iy*x_num + ix;
			if(0 <= pos and pos < map.data.size())	
				_free &= (map.data[iy*x_num + ix] == 0);
		}
}
