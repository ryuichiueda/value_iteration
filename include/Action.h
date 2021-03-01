#ifndef VALUE_ACTION_
#define VALUE_ACTION_

#include "StateTransition.h"
#include <vector>
#include <string>
using namespace std;

class Action{
public:
	string _name;
	double _delta_fw;  //forward traveling distance[m]
	double _delta_rot;  //rotation[deg]

//	double _delta_fw_stdev;
//	double _delta_rot_stdev;

	vector< vector<StateTransition> > _state_transitions; //thetaごとに状態遷移先のリストを保存

	Action(string name, double fw, double rot);
};

#endif
