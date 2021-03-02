#include "ValueIterator.h"
using namespace std;

State::State(int x, int y, int theta, int map_value)
{
	_ix = x;
	_iy = y;
	_it = theta;
	_cost = ValueIterator::_max_cost;
	_free = (map_value == 0);
	_final_state = false;
	_optimal_action = NULL;
}
