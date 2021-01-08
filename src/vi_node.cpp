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
	value_iteration::ViFeedback vi_feedback;
	value_iteration::ViResult vi_result;
	
	actionlib::SimpleActionServer<value_iteration::ViAction> as;

	ViAction(ros::NodeHandle &h, ValueIterator &vi) : as(h, "vi_controller", boost::bind(&ViAction::executeVi, this, _1), false)
	{
		as.start();
		ROS_INFO("ViAction started");
	}
	
	void executeVi(const value_iteration::ViGoalConstPtr &goal)
	{
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
	value_iterator.outputPbmMap();

	ros::spin();

	return 0;
}
