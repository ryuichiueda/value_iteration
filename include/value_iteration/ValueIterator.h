#ifndef VALUE_ITERATOR_
#define VALUE_ITERATOR_

#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include <grid_map_msgs/GetGridMap.h>
#include <vector>
#include <fstream>

#include "SweepWorkerStatus.h"
#include "Action.h"
#include "State.h"

namespace value_iteration{

using namespace std;


class ValueIterator{
private: 
	vector<State> _states;
	vector<Action> _actions;

	double _cell_x_width, _cell_y_width, _cell_t_width;
	int _cell_x_num, _cell_y_num, _cell_t_num;
	int _center_state_ix, _center_state_iy;

	double map_origin_x_;
	double map_origin_y_;

	double _final_state_x, _final_state_y, _final_state_width;
	//uint64_t _delta;

	const static int _resolution_x, _resolution_y, _resolution_t;
	const static unsigned char _resolution_x_bit, _resolution_y_bit, _resolution_t_bit;

	void accurateStateTransition(Action &a, double from_x, double from_y, double from_t, double &to_x, double &to_y, double &to_t);
	void toCellPos(double x, double y, double t, int &ix, int &iy, int &it);

	void setAction(XmlRpc::XmlRpcValue &action_list);
	void setState(const nav_msgs::OccupancyGrid &map, double safety_radius);
	void setStateValues(void);
	void setStateTransition(void);
	void setStateTransition(Action &a, int it);
	void setStateTransitionWorker(int it);

	uint64_t valueIteration(State &s);
	uint64_t actionCost(State &s, Action &a);

	int toIndex(int ix, int iy, int it);
public: 
	ValueIterator(nav_msgs::OccupancyGrid &map, XmlRpc::XmlRpcValue &params);

	Action *posToAction(double x, double y, double t_rad);

	void outputPbmMap(void);

	void initialize(double goal_x, double goal_y);

	void valueIterationWorker(int times, int id);
	map<int, SweepWorkerStatus> _status; 

	bool actionImageWriter(grid_map_msgs::GetGridMap::Response& response);
	bool outputValuePgmMap(grid_map_msgs::GetGridMap::Response& response);

	const static uint64_t _max_cost;
	const static uint64_t _prob_base;
	const static unsigned char _prob_base_bit;
};

const unsigned char ValueIterator::_resolution_x_bit = 6;
const unsigned char ValueIterator::_resolution_y_bit = 6;
const unsigned char ValueIterator::_resolution_t_bit = 6;
const int ValueIterator::_resolution_x = 1<<ValueIterator::_resolution_x_bit;
const int ValueIterator::_resolution_y = 1<<ValueIterator::_resolution_y_bit;
const int ValueIterator::_resolution_t = 1<<ValueIterator::_resolution_t_bit;
const unsigned char ValueIterator::_prob_base_bit = ValueIterator::_resolution_x_bit+ValueIterator::_resolution_y_bit+ValueIterator::_resolution_t_bit;
const uint64_t ValueIterator::_prob_base = 1<<ValueIterator::_prob_base_bit;
const uint64_t ValueIterator::_max_cost = 1000000000*ValueIterator::_prob_base;

}

#endif
