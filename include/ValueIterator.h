#ifndef VALUE_ITERATOR_
#define VALUE_ITERATOR_

#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include <vector>
#include <fstream>
using namespace std;

class SweepWorkerStatus{
public: 	
	bool _finished;
	int _sweep_step;

	SweepWorkerStatus();
};

class State{
public: 
//	double _value;
	int64_t _ivalue;
	int _ix, _iy, _it;
	bool _free;
	bool _final_state;

	State(int x, int y, int theta, int map_value);
};

class StateTransition{
public:
	int _dix, _diy, _dit;
	int _prob;

	StateTransition(int dix, int diy, int dit, int prob);
	string to_string(void);
};

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

class ValueIterator{
private: 
	vector<State> _states;
	vector<Action> _actions;

	double _cell_x_width, _cell_y_width, _cell_t_width;
	int _cell_x_num, _cell_y_num, _cell_t_num;
	int _center_state_ix, _center_state_iy;
	double _final_state_x, _final_state_y, _final_state_width;
	double _delta;

	const static int _resolution_x, _resolution_y, _resolution_t;

	void accurateStateTransition(Action &a, double from_x, double from_y, double from_t, double &to_x, double &to_y, double &to_t);
	void toCellPos(double x, double y, double t, int &ix, int &iy, int &it);

	void setAction(void);
	void setStateValues(void);
	void setStateTransition(void);
	void setStateTransition(Action &a, int it);
	void setStateTransitionWorker(int it);

	double valueIteration(State &s);
	int64_t actionValue(State &s, Action &a);

	int toIndex(int ix, int iy, int it);
public: 
	ValueIterator(nav_msgs::OccupancyGrid &map);

	void outputPbmMap(void);
	void outputValuePgmMap(void);

	void valueIterationWorker(int times, int id);
	map<int, SweepWorkerStatus> _status; 

	const static int64_t _value_min;
	const static int64_t _prob_base;
};

const int ValueIterator::_resolution_x = 100;
const int ValueIterator::_resolution_y = 100;
const int ValueIterator::_resolution_t = 100;
const int64_t ValueIterator::_prob_base = ValueIterator::_resolution_x*ValueIterator::_resolution_y*ValueIterator::_resolution_t;
const int64_t ValueIterator::_value_min = -1000000000*ValueIterator::_prob_base;

#endif
