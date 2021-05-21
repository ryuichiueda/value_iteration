#ifndef VALUE_ITERATOR_
#define VALUE_ITERATOR_

#include "ros/ros.h"
#include "std_msgs/UInt32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include <grid_map_msgs/GetGridMap.h>
#include <vector>
#include <fstream>

#include "SweepWorkerStatus.h"
#include "Action.h"
#include "State.h"

namespace value_iteration{

class ValueIterator{
private: 
	std::vector<State> states_;
	std::vector<Action> &actions_;

	double xy_resolution_, t_resolution_;
	int cell_num_x_, cell_num_y_, cell_num_t_;

	double map_origin_x_;
	double map_origin_y_;

	const static unsigned char resolution_xy_bit_, resolution_t_bit_;

	void noNoiseStateTransition(Action &a, double from_x, double from_y, double from_t, double &to_x, double &to_y, double &to_t);

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
	ValueIterator(nav_msgs::OccupancyGrid &map, vector<Action> &actions,
		int theta_cell_num, int thread_num,
		double safety_radius, double safety_radius_penalty,
		double goal_margin_radius, int goal_margin_theta);

	Action *posToAction(double x, double y, double t_rad);

	void outputPbmMap(void);

	void setGoal(double goal_x, double goal_y, int goal_t);

	void valueIterationWorker(int times, int id);
	map<int, SweepWorkerStatus> status_; 

	bool actionImageWriter(grid_map_msgs::GetGridMap::Response& response);
	bool outputValuePgmMap(grid_map_msgs::GetGridMap::Response& response);

	double goal_x_, goal_y_, goal_margin_radius_;
	int goal_t_, goal_margin_theta_;
	int thread_num_;

	const static uint64_t max_cost_;
	const static uint64_t prob_base_;
	const static unsigned char prob_base_bit_;

	bool finished(std_msgs::UInt32MultiArray &sweep_times, std_msgs::Float32MultiArray &deltas);
};

const unsigned char ValueIterator::resolution_xy_bit_ = 6;
const unsigned char ValueIterator::resolution_t_bit_ = 6;
const unsigned char ValueIterator::prob_base_bit_ = ValueIterator::resolution_xy_bit_*2+ValueIterator::resolution_t_bit_;
const uint64_t ValueIterator::prob_base_ = 1<<ValueIterator::prob_base_bit_;
const uint64_t ValueIterator::max_cost_ = 1000000000*ValueIterator::prob_base_;

}

#endif
