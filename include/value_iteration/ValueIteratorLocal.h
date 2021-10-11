#ifndef VALUE_ITERATOR_LOCAL_
#define VALUE_ITERATOR_LOCAL_

#include "ValueIterator.h"

namespace value_iteration{

class ValueIteratorLocal : public ValueIterator{
public:
	ValueIteratorLocal(vector<Action> &actions, int thread_num);

	uint64_t valueIterationLocal(State &s);
	void localValueIterationWorker(int id);

	void setMapWithOccupancyGrid(nav_msgs::OccupancyGrid &map, int theta_cell_num,
		double safety_radius, double safety_radius_penalty,
		double goal_margin_radius, int goal_margin_theta);

	Action *posToAction(double x, double y, double t_rad);

	void setLocalCost(const sensor_msgs::LaserScan::ConstPtr &msg, double x, double y, double t);
	void setLocalWindow(double x, double y);
	void makeLocalValueFunctionMap(nav_msgs::OccupancyGrid &map, int threshold, 
			double x, double y, double yaw_rad);

	void copyFromGlobal(void);
private: 
	void localValueIterationLoop1(void);
	void localValueIterationLoop2(void);

	bool inLocalArea(int ix, int iy);
	uint64_t actionCostLocal(State &s, Action &a);

	int local_ix_min_, local_ix_max_, local_iy_min_, local_iy_max_;
	int local_ixy_range_;
	double local_xy_range_;
};

}

#endif
