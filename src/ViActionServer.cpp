#include "ViActionServer.h"
#include <thread>
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
	vector<thread> ths;
	for(int t=0; t<goal->threadnum; t++)
		ths.push_back(thread(&ValueIterator::valueIterationWorker, &_vi, goal->sweepnum));

	for(auto &th : ths){
		th.join();
	}

		/*
	outputValuePgmMap();
	exit(0);

		sleep(1);
		ROS_INFO("%d, %d", sweepnum, threadnum);
		value_iteration::ViFeedback vi_feedback;
		vi_feedback.current_sweep_time = i;
		as.publishFeedback(vi_feedback);
	*/


	value_iteration::ViResult vi_result;
	vi_result.finished = false;
	as.setSucceeded(vi_result);
}
