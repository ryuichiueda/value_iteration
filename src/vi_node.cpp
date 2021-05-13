#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <value_iteration/ViAction.h>

#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ValueIterator.h"
#include "ViActionServer.h"
#include <iostream>
#include <vector>

#include <grid_map_msgs/GridMap.h>
using namespace std;

class ViNode{
public:
	ViNode(){}
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

	XmlRpc::XmlRpcValue vi_node;
	n.getParam("/vi_node", vi_node);
	ROS_ASSERT(vi_node.getType() == XmlRpc::XmlRpcValue::TypeStruct);

	ros::Publisher publisher = n.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

	ValueIterator value_iterator(res.map, vi_node);
	ViActionServer vi_server(n, value_iterator);

	ros::Rate loop_rate(1);
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::spin();

	return 0;
}
