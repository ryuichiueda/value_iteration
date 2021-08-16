#include "value_iteration/Action.h"

namespace value_iteration{

Action::Action(std::string name, double fw, double rot, double fw_dev, double rot_dev, int id) : id_(id)
{
	_name = name;

	_delta_fw = fw;
	_delta_rot = rot;

	_delta_fw_stddev = fw_dev;
	_delta_rot_stddev = rot_dev;

	_fw_gen.reset(new std::normal_distribution<> (fw, fw_dev));
	_rot_gen.reset(new std::normal_distribution<> (rot, rot_dev));

        std::random_device seed_gen;
	_random_engine = std::default_random_engine(seed_gen());
}

}
