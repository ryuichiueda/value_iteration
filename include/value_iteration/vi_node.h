#ifndef _VI_NODE_H__
#define _VI_NODE_H__

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <value_iteration/ViAction.h>

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "value_iteration/ValueIterator.h"
#include <iostream>
#include <vector>
#include <thread>

#include <grid_map_msgs/GetGridMap.h>
#include <std_msgs/UInt32MultiArray.h>
#include <tf/tf.h>

namespace value_iteration{

class ViNode{

public:
	ViNode();
	~ViNode();

	void pubValueFunction(void);
private:
	vector<Action> *actions_;
	shared_ptr<ValueIterator> vi_;
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	ros::ServiceServer srv_policy_;
	ros::ServiceServer srv_value_;

	ros::Publisher pub_cmd_vel_;
	ros::Publisher pub_value_function_;
	ros::Subscriber sub_pose_;

	shared_ptr<actionlib::SimpleActionServer<value_iteration::ViAction> > as_;

	void executeVi(const value_iteration::ViGoalConstPtr &goal);
	bool servePolicy(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response);
	bool serveValue(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response);

	void poseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

	void setActions(void);
	void setMap(nav_msgs::GetMap::Response &res);
	void setCommunication(void);

	double x_, y_, yaw_;

	string status_;
	bool online_;
};

}
#endif
