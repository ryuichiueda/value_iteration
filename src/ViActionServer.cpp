#include "ViActionServer.h"
using namespace std;

ViActionServer::ViActionServer(ros::NodeHandle &h, ValueIterator &vi) 
	: as(h, "vi_controller", boost::bind(&ViActionServer::executeVi, this, _1), false),
	  _vi(vi)
{
	as.start();
	ROS_INFO("ViActionServer started");
}


void ViActionServer::executeVi(const value_iteration::ViGoalConstPtr &goal)
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
