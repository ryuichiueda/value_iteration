#ifndef VALUE_ITERATOR_
#define VALUE_ITERATOR_

#include "ros/ros.h"
#include "std_msgs/UInt32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
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
	geometry_msgs::Quaternion map_origin_quat_;

	const static unsigned char resolution_xy_bit_, resolution_t_bit_;

	void noNoiseStateTransition(Action &a, double from_x, double from_y, double from_t, double &to_x, double &to_y, double &to_t);

	void setState(const nav_msgs::OccupancyGrid &map, double safety_radius, double safety_radius_penalty);
	void setStateValues(void);

	void setStateTransition(void);
	void setStateTransitionWorker(int it);
	void setStateTransitionWorkerSub(Action &a, int it);
	void cellDelta(double x, double y, double t, int &ix, int &iy, int &it);

	uint64_t valueIteration(State &s);
	uint64_t valueIterationLocal(State &s);
	uint64_t actionCost(State &s, Action &a);
	uint64_t actionCostLocal(State &s, Action &a);

	int toIndex(int ix, int iy, int it);
	bool inMapArea(int ix, int iy);
	bool inLocalArea(int ix, int iy);

	int local_ix_min_, local_ix_max_, local_iy_min_, local_iy_max_;
	int local_ixy_range_;
	double local_xy_range_;
public: 
	ValueIterator(vector<Action> &actions, int thread_num);

	Action *posToAction(double x, double y, double t_rad, bool &goal);
	Action *posToActionLocal(double x, double y, double t_rad, bool &goal);

	void outputPbmMap(void);

	void setMapWithOccupancyGrid(nav_msgs::OccupancyGrid &map, int theta_cell_num,
		double safety_radius, double safety_radius_penalty,
		double goal_margin_radius, int goal_margin_theta);

	void setMapWithCostGrid(nav_msgs::OccupancyGrid &map, int theta_cell_num,
		double safety_radius, double safety_radius_penalty,
		double goal_margin_radius, int goal_margin_theta);

	void setGoal(double goal_x, double goal_y, int goal_t);

	void valueIterationWorker(int times, int id);
	void localValueIterationWorker(void);
	map<int, SweepWorkerStatus> status_; 

	bool policyWriter(grid_map_msgs::GetGridMap::Response& response);
	bool valueFunctionWriter(grid_map_msgs::GetGridMap::Response& response);

	void makeValueFunctionMap(nav_msgs::OccupancyGrid &map,
			double x, double y, double yaw_rad);
	void makeLocalValueFunctionMap(nav_msgs::OccupancyGrid &map,
			double x, double y, double yaw_rad);

	bool finished(std_msgs::UInt32MultiArray &sweep_times, std_msgs::Float32MultiArray &deltas);

	void setLocalCost(const sensor_msgs::LaserScan::ConstPtr &msg, double x, double y, double t);

	double goal_x_, goal_y_, goal_margin_radius_;
	int goal_t_, goal_margin_theta_;
	int thread_num_;

	const static uint64_t max_cost_;
	const static uint64_t prob_base_;
	const static unsigned char prob_base_bit_;
};

const unsigned char ValueIterator::resolution_xy_bit_ = 6;
const unsigned char ValueIterator::resolution_t_bit_ = 6;
const unsigned char ValueIterator::prob_base_bit_ = ValueIterator::resolution_xy_bit_*2+ValueIterator::resolution_t_bit_;
const uint64_t ValueIterator::prob_base_ = 1<<ValueIterator::prob_base_bit_;
const uint64_t ValueIterator::max_cost_ = 1000000000*ValueIterator::prob_base_;

}

#endif
