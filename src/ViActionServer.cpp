#include "ViActionServer.h"
#include <std_msgs/UInt32MultiArray.h>
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
	int sweepnum = goal->sweepnum > 0 ? goal->sweepnum : INT_MAX;

	vector<thread> ths;
	for(int t=0; t<goal->threadnum; t++)
		ths.push_back(thread(&ValueIterator::valueIterationWorker, &_vi, sweepnum, t));

	value_iteration::ViFeedback vi_feedback;
	vi_feedback.current_sweep_times.data.resize(goal->threadnum);
	vi_feedback.deltas.data.resize(goal->threadnum);
	while(1){
		sleep(10);
		for(int t=0; t<goal->threadnum; t++){
			vi_feedback.current_sweep_times.data[t] = _vi._status[t]._sweep_step;
			vi_feedback.deltas.data[t] = _vi._status[t]._delta;
		}
		as.publishFeedback(vi_feedback);

		bool finish = true;
		for(int t=0; t<goal->threadnum; t++)
			finish &= _vi._status[t]._finished;
		if(finish)
			break;

		_vi.outputValuePgmMap();
	}

	for(auto &th : ths)
		th.join();

	_vi.outputValuePgmMap();

	value_iteration::ViResult vi_result;
	vi_result.finished = true;
	as.setSucceeded(vi_result);
}
