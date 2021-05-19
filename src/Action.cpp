#include "value_iteration/Action.h"

namespace value_iteration{

Action::Action(std::string name, double fw, double rot, int id) : id_(id)
{
	_name = name;

	_delta_fw = fw;
	_delta_rot = rot;

	//_delta_fw_stdev = fabs(fw)*0.1;
	//_delta_rot_stdev = fabs(rot)*0.1;
}

}
