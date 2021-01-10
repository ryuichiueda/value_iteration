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

class ViServer{

public:
	ValueIterator& _vi;
	
	actionlib::SimpleActionServer<value_iteration::ViAction> as;

	ViServer(ros::NodeHandle &h, ValueIterator &vi);
	void executeVi(const value_iteration::ViGoalConstPtr &goal);
};

ViServer::ViServer(ros::NodeHandle &h, ValueIterator &vi) 
	: as(h, "vi_controller", boost::bind(&ViServer::executeVi, this, _1), false),
	  _vi(vi)
{
	as.start();
	ROS_INFO("ViServer started");
}


void ViServer::executeVi(const value_iteration::ViGoalConstPtr &goal)
{
	for(int i=0;i<=10;i++){
		sleep(1);
		ROS_INFO("send feedback");
		value_iteration::ViFeedback vi_feedback;
		vi_feedback.current_sweep_time = i;
		as.publishFeedback(vi_feedback);
	}

	value_iteration::ViResult vi_result;
	vi_result.finished = false;
	as.setSucceeded(vi_result);
}

#endif
