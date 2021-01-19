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
	bool fin = false;
	for(int t=0; t<goal->threadnum; t++)
		ths.push_back(thread(&ValueIterator::valueIterationWorker, &_vi, goal->sweepnum, t));

	while(1){
		sleep(10);

		value_iteration::ViFeedback vi_feedback;
		vi_feedback.current_sweep_time = 3; //いまのところてきとう
		as.publishFeedback(vi_feedback);

		bool finish = true;
		for(int t=0; t<goal->threadnum; t++){
			ROS_INFO("thread%d: %d", t, _vi._finished[t]);
			finish &= _vi._finished[t];
		}
		if(finish)
			break;
	}

	for(auto &th : ths){
		th.join();
	}


	value_iteration::ViResult vi_result;
	vi_result.finished = false;
	as.setSucceeded(vi_result);
}
