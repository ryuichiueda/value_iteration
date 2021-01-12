#ifndef VI_SERVER_H__
#define VI_SERVER_H__

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <value_iteration/ViAction.h>

#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ValueIterator.h"
#include <iostream>
#include <vector>
using namespace std;

class ViActionServer{

public:
	ValueIterator& _vi;
	
	actionlib::SimpleActionServer<value_iteration::ViAction> as;

	ViActionServer(ros::NodeHandle &h, ValueIterator &vi);
	void executeVi(const value_iteration::ViGoalConstPtr &goal);
};

#endif
