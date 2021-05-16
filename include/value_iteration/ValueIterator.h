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
	vector<State> states_;
	vector<Action> actions_;

	double xy_resolution_, t_resolution_;
	int cell_num_x_, cell_num_y_, cell_num_t_;

	double map_origin_x_;
	double map_origin_y_;

	const static unsigned char resolution_xy_bit_, resolution_t_bit_;

	void accurateStateTransition(Action &a, double from_x, double from_y, double from_t, double &to_x, double &to_y, double &to_t);

	void setAction(XmlRpc::XmlRpcValue &action_list);
	void setState(const nav_msgs::OccupancyGrid &map, double safety_radius);
	void setStateValues(void);

	void setStateTransition(void);
	void setStateTransitionWorker(int it);
	void setStateTransitionWorkerSub(Action &a, int it);
	void cellDelta(double x, double y, double t, int &ix, int &iy, int &it);

	uint64_t valueIteration(State &s);
	uint64_t actionCost(State &s, Action &a);

	int toIndex(int ix, int iy, int it);
public: 
	ValueIterator(nav_msgs::OccupancyGrid &map, XmlRpc::XmlRpcValue &params, int theta_cell_num, int thread_num);

	Action *posToAction(double x, double y, double t_rad);

	void outputPbmMap(void);

	void setGoal(double goal_x, double goal_y);

	void valueIterationWorker(int times, int id);
	map<int, SweepWorkerStatus> status_; 

	bool actionImageWriter(grid_map_msgs::GetGridMap::Response& response);
	bool outputValuePgmMap(grid_map_msgs::GetGridMap::Response& response);

	double goal_x_, goal_y_, goal_width_;
	int thread_num_;

	const static uint64_t max_cost_;
	const static uint64_t prob_base_;
	const static unsigned char prob_base_bit_;
};

const unsigned char ValueIterator::resolution_xy_bit_ = 6;
const unsigned char ValueIterator::resolution_t_bit_ = 6;
const unsigned char ValueIterator::prob_base_bit_ = ValueIterator::resolution_xy_bit_+ValueIterator::resolution_xy_bit_+ValueIterator::resolution_t_bit_;
const uint64_t ValueIterator::prob_base_ = 1<<ValueIterator::prob_base_bit_;
const uint64_t ValueIterator::max_cost_ = 1000000000*ValueIterator::prob_base_;

}

#endif