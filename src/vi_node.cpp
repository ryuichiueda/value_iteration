#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <value_iteration/ViAction.h>

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ValueIterator.h"
#include <iostream>
#include <vector>
#include <thread>

#include <grid_map_msgs/GetGridMap.h>
#include <std_msgs/UInt32MultiArray.h>
using namespace std;

class ViNode{

public:
	shared_ptr<ValueIterator> vi_;
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	ros::ServiceServer srv_policy_;
	ros::ServiceServer srv_value_;

	ros::Publisher pub_cmd_vel_;
	ros::Subscriber sub_pose_;

	shared_ptr<actionlib::SimpleActionServer<value_iteration::ViAction> > as_;

	ViNode();

	void executeVi(const value_iteration::ViGoalConstPtr &goal);
	bool servePolicy(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response);
	bool serveValue(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response);

	void poseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
};


ViNode::ViNode() : private_nh_("~") 
{
	while(!ros::service::waitForService("/static_map", ros::Duration(3.0))){
		ROS_INFO("Waiting for static_map");
	}

	ros::ServiceClient client = nh_.serviceClient<nav_msgs::GetMap>("/static_map");

	nav_msgs::GetMap::Request req;
	nav_msgs::GetMap::Response res;
	if(not client.call(req, res)){
		ROS_ERROR("static_map not working");
		exit(1);
	}

	bool online;
	private_nh_.param("online", online, false);
	ROS_INFO("BOOL: %d", (int)online);
	if(online){
		ROS_INFO("SET ONLINE");
		pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 2, true);
		sub_pose_ = nh_.subscribe("mcl_pose", 2, &ViNode::poseReceived, this);
	}

	XmlRpc::XmlRpcValue params;
	nh_.getParam("/vi_node", params);
	ROS_ASSERT(params.getType() == XmlRpc::XmlRpcValue::TypeStruct);

	vi_.reset(new ValueIterator(res.map, params));
	as_.reset(new actionlib::SimpleActionServer<value_iteration::ViAction>( nh_, "vi_controller", boost::bind(&ViNode::executeVi, this, _1), false));
	as_->start();

	srv_policy_ = nh_.advertiseService("/policy", &ViNode::servePolicy, this);
	srv_value_ = nh_.advertiseService("/value", &ViNode::serveValue, this);
}

void ViNode::poseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
	ROS_INFO("POSE: %f, %f", msg->pose.pose.position.x, msg->pose.pose.position.y);
	Action *a = vi_->posToAction(msg->pose.pose.position.x,
				msg->pose.pose.position.y, msg->pose.pose.orientation.z);

	if(a == NULL)
		return;
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = a->_delta_fw;
	cmd_vel.angular.z= a->_delta_rot/180*M_PI;

	pub_cmd_vel_.publish(cmd_vel);
}

bool ViNode::servePolicy(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response)
{
	vi_->actionImageWriter(response);
	return true;
}

bool ViNode::serveValue(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response)
{
	vi_->outputValuePgmMap(response);
	return true;
}

void ViNode::executeVi(const value_iteration::ViGoalConstPtr &goal)
{
	int sweepnum = goal->sweepnum > 0 ? goal->sweepnum : INT_MAX;

	vector<thread> ths;
	for(int t=0; t<goal->threadnum; t++)
		ths.push_back(thread(&ValueIterator::valueIterationWorker, vi_.get(), sweepnum, t));

	value_iteration::ViFeedback vi_feedback;
	vi_feedback.current_sweep_times.data.resize(goal->threadnum);
	vi_feedback.deltas.data.resize(goal->threadnum);
	while(1){
		sleep(1);
		for(int t=0; t<goal->threadnum; t++){
			vi_feedback.current_sweep_times.data[t] = vi_->_status[t]._sweep_step;
			vi_feedback.deltas.data[t] = vi_->_status[t]._delta;
		}
		as_->publishFeedback(vi_feedback);

		bool finish = true;
		for(int t=0; t<goal->threadnum; t++)
			finish &= vi_->_status[t]._finished;
		if(finish)
			break;
	}

	for(auto &th : ths)
		th.join();


	value_iteration::ViResult vi_result;
	vi_result.finished = true;
	as_->setSucceeded(vi_result);
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"vi_node");

	ViNode vi_node;

	ros::Rate loop_rate(1);
	while(ros::ok()){
		ROS_INFO("LOOP");
		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::spin();

	return 0;
}
