#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <value_iteration/ViAction.h>

#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ValueIterator.h"
#include "ViActionServer.h"
#include <iostream>
#include <vector>
using namespace std;

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

	XmlRpc::XmlRpcValue action_list;
	n.getParam("/vi_node/action_list", action_list);

	/*
	ROS_ASSERT(action_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

	for(int i=0; i<action_list.size(); i++){
		cout << action_list[i]["name"] << endl;
		cout << action_list[i]["onestep_forward_m"] << endl;
		cout << action_list[i]["onestep_rotation_deg"] << endl;

		//ROS_ASSERT(action_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
		//sum += static_cast<double>(action_list[i]);
	}
	*/

	ValueIterator value_iterator(res.map, action_list);
	ViActionServer vi_server(n, value_iterator);

	ros::spin();

	return 0;
}
