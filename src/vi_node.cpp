#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <value_iteration/ViAction.h>

#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ValueIterator.h"
#include <iostream>
#include <vector>

#include <grid_map_msgs/GridMap.h>

#include <std_msgs/UInt32MultiArray.h>
#include <thread>
using namespace std;

class ViNode{

public:
	shared_ptr<ValueIterator> vi_;
	ros::NodeHandle nh_;

	void setAction(void);
	
	shared_ptr<actionlib::SimpleActionServer<value_iteration::ViAction> > as;

	ViNode();
	void executeVi(const value_iteration::ViGoalConstPtr &goal);
};


ViNode::ViNode()
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

	XmlRpc::XmlRpcValue params;
	nh_.getParam("/vi_node", params);
	ROS_ASSERT(params.getType() == XmlRpc::XmlRpcValue::TypeStruct);

	vi_.reset(new ValueIterator(res.map, params));
}

void ViNode::setAction(void)
{
	as.reset(new actionlib::SimpleActionServer<value_iteration::ViAction>(nh_, "vi_controller", boost::bind(&ViNode::executeVi, this, _1), false));
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
		as->publishFeedback(vi_feedback);

		bool finish = true;
		for(int t=0; t<goal->threadnum; t++)
			finish &= vi_->_status[t]._finished;
		if(finish)
			break;

		//vi_.outputValuePgmMap();
	}

	for(auto &th : ths)
		th.join();

	vi_->outputValuePgmMap();
	vi_->actionImageWriter();

	value_iteration::ViResult vi_result;
	vi_result.finished = true;
	as->setSucceeded(vi_result);
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"vi_node");

	ViNode vi_node;

	vi_node.setAction();
	vi_node.as->start();
	ROS_INFO("ViNode started");

	ros::Rate loop_rate(1);
	while(ros::ok()){
		ROS_INFO("LOOP");
		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::spin();

	return 0;
}

