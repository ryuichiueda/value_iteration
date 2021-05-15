#include "value_iteration/ValueIterator.h"
#include <thread>

namespace value_iteration{

using namespace std;

SweepWorkerStatus::SweepWorkerStatus()
{
	_finished = false;
	_sweep_step = 0;
	_delta = ValueIterator::_max_cost;
}

}
