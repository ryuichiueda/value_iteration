#ifndef VALUE_ITERATOR_
#define VALUE_ITERATOR_

#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include <vector>
#include <fstream>
using namespace std;

class State{
public: 
	double _value;
	int _ix, _iy, _it;
	bool _free;
	bool _final_state;

	State(int x, int y, int theta, int map_value);
};

class StateTransition{
public:
	int _dix, _diy, _dit;
	double _prob;

	StateTransition(int dix, int diy, int dit, double prob);
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

	void accurateStateTransition(Action &a, double from_x, double from_y, double from_t, double &to_x, double &to_y, double &to_t);
	void toCellPos(double x, double y, double t, int &ix, int &iy, int &it);

	void setAction(void);
	void setStateValues(void);
	void setStateTransition(void);
	void setStateTransition(Action &a, int it);
	void setStateTransitionWorker(int it);

	double valueIteration(State &s);
	double actionValue(State &s, Action &a);

	int toIndex(int ix, int iy, int it);
public: 
	ValueIterator(nav_msgs::OccupancyGrid &map);

	void outputPbmMap(void);
	void outputValuePgmMap(void);

	void valueIterationWorker(int times);

	const static double _value_min;
};

const double ValueIterator::_value_min = -100000000.0;

#endif
