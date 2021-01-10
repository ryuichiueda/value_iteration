#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <value_iteration/ViAction.h>

#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ValueIterator.h"
#include <iostream>
#include <vector>
using namespace std;

class ViAction{

public:
	ValueIterator& _vi;
	
	actionlib::SimpleActionServer<value_iteration::ViAction> as;

	ViAction(ros::NodeHandle &h, ValueIterator &vi) : as(h, "vi_controller", boost::bind(&ViAction::executeVi, this, _1), false),
		_vi(vi)
	{
		as.start();
		ROS_INFO("ViAction started");
	}
	
	void executeVi(const value_iteration::ViGoalConstPtr &goal)
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
};

int main(int argc, char **argv)
{
	ros::init(argc,argv,"vi_node");
	ros::NodeHandle n;

	while(!ros::service::waitForService("/static_map", ros::Duration(3.0))){
		ROS_INFO("Waiting for static_map");
	}

	ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("/static_map");

	nav_msgs::GetMap::Request req;
	nav_msgs::GetMap::Response res;
	if(not client.call(req, res)){
		ROS_ERROR("static_map not working");
		return 1;
	}

	ValueIterator value_iterator(res.map);
	ViAction vi_action(n, value_iterator);
//	value_iterator.outputPbmMap();

	ros::spin();

	return 0;
}
