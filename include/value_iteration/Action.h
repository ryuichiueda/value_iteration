#ifndef VALUE_ACTION_
#define VALUE_ACTION_

#include "StateTransition.h"
#include <vector>
#include <string>
#include <memory>
#include <random>

namespace value_iteration{

class Action{
public:
	Action(std::string name, double fw, double rot, double fw_dev, double rot_dev, int id);

	std::string _name;
	double _delta_fw;  //forward traveling distance[m]
	double _delta_rot;  //rotation[deg]

	int id_;

	double _delta_fw_stddev; //standard deviation[m]
	double _delta_rot_stddev; //standard deviation[deg]

	std::shared_ptr< std::normal_distribution<> > _fw_gen; //generator of fw velocity with noise
	std::shared_ptr< std::normal_distribution<> > _rot_gen; //generator of rot velocity with noise

        std::default_random_engine _random_engine;

	std::vector< std::vector<StateTransition> > _state_transitions; //thetaごとに状態遷移先のリストを保存
};

}

#endif

