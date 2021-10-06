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
/* value iteration */
	std::vector<State> states_;
	std::vector<Action> &actions_;
	std::vector<std::vector<int> > sweep_orders_;

	uint64_t valueIteration(State &s);
	uint64_t actionCost(State &s, Action &a);
public:
	void setGoal(double goal_x, double goal_y, int goal_t);
	void valueIterationWorker(int times, int id);

/* calculation */
private: 
	int toIndex(int ix, int iy, int it);
	bool inMapArea(int ix, int iy);
	void cellDelta(double x, double y, double t, int &ix, int &iy, int &it);
	void noNoiseStateTransition(Action &a, double from_x, double from_y, double from_t, double &to_x, double &to_y, double &to_t);

/* robot control */
public: 
	Action *posToActionLocal(double x, double y, double t_rad);
	bool endOfTrial(void);
	bool arrived(void);

/* initialization */
	ValueIterator(vector<Action> &actions, int thread_num);
	void setMapWithOccupancyGrid(nav_msgs::OccupancyGrid &map, int theta_cell_num,
		double safety_radius, double safety_radius_penalty,
		double goal_margin_radius, int goal_margin_theta);

	void setMapWithCostGrid(nav_msgs::OccupancyGrid &map, int theta_cell_num,
		double safety_radius, double safety_radius_penalty,
		double goal_margin_radius, int goal_margin_theta);
private:
	void setState(const nav_msgs::OccupancyGrid &map, double safety_radius, double safety_radius_penalty);
	void setStateValues(void);
	void setStateTransition(void);
	void setStateTransitionWorker(int it);
	void setStateTransitionWorkerSub(Action &a, int it);
	void setSweepOrders(void);

/* ros output */
public:
	bool policyWriter(grid_map_msgs::GetGridMap::Response& response);
	bool valueFunctionWriter(grid_map_msgs::GetGridMap::Response& response);
	void makeValueFunctionMap(nav_msgs::OccupancyGrid &map, int threshold,
			double x, double y, double yaw_rad);

/* control of value iteration threads */
	map<int, SweepWorkerStatus> thread_status_; 
	bool finished(std_msgs::UInt32MultiArray &sweep_times, std_msgs::Float32MultiArray &deltas);
	void setCancel(void);
private:
	string status_;

/* parameters */
public: 
	double goal_x_, goal_y_, goal_margin_radius_;
	int goal_t_, goal_margin_theta_;
	int thread_num_;
	const static uint64_t max_cost_;
	const static uint64_t prob_base_;
	const static unsigned char prob_base_bit_;
private:
	double xy_resolution_, t_resolution_;
	int cell_num_x_, cell_num_y_, cell_num_t_;
	double map_origin_x_;
	double map_origin_y_;
	geometry_msgs::Quaternion map_origin_quat_;
	const static unsigned char resolution_xy_bit_, resolution_t_bit_;

/* for local value iteration */
public: 
	void localValueIterationWorker(int id);
	void localValueIterationLoop1(void);
	void localValueIterationLoop2(void);
	void makeLocalValueFunctionMap(nav_msgs::OccupancyGrid &map, int threshold, 
			double x, double y, double yaw_rad);
	void setLocalWindow(double x, double y);
	void setLocalCost(const sensor_msgs::LaserScan::ConstPtr &msg, double x, double y, double t);
private:
	uint64_t valueIterationLocal(State &s);
	uint64_t actionCostLocal(State &s, Action &a);
	bool inLocalArea(int ix, int iy);

	int local_ix_min_, local_ix_max_, local_iy_min_, local_iy_max_;
	int local_ixy_range_;
	double local_xy_range_;
};

const unsigned char ValueIterator::resolution_xy_bit_ = 6;
const unsigned char ValueIterator::resolution_t_bit_ = 6;
const unsigned char ValueIterator::prob_base_bit_ = ValueIterator::resolution_xy_bit_*2+ValueIterator::resolution_t_bit_;
const uint64_t ValueIterator::prob_base_ = 1<<ValueIterator::prob_base_bit_;
const uint64_t ValueIterator::max_cost_ = 1000000000*ValueIterator::prob_base_;

}

#endif
