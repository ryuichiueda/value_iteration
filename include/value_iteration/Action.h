#ifndef VALUE_ACTION_
#define VALUE_ACTION_

#include "StateTransition.h"
#include <vector>
#include <string>

namespace value_iteration{

class Action{
public:
	Action(std::string name, double fw, double rot, int id);

	std::string _name;
	double _delta_fw;  //forward traveling distance[m]
	double _delta_rot;  //rotation[deg]

	int id_;

//	double _delta_fw_stdev;
//	double _delta_rot_stdev;

	std::vector< std::vector<StateTransition> > _state_transitions; //thetaごとに状態遷移先のリストを保存
};

}

#endif

