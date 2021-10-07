#include "value_iteration/ValueIteratorLocal.h"

namespace value_iteration{

ValueIteratorLocal::ValueIteratorLocal(std::vector<Action> &actions, int thread_num) : ValueIterator(actions, thread_num)
{
}

void ValueIteratorLocal::localValueIterationWorker(int id)
{
	while(status_ == "canceled" or status_ == "goal"){
		ROS_INFO("STATUS PROBLEM: ", status_.c_str());
		status_ = "executing";
	}

	while(status_ != "canceled" and status_ != "goal"){
		if(id%2){
			localValueIterationLoop1();
		}else{
			localValueIterationLoop2();
		}
	}
}

void ValueIteratorLocal::localValueIterationLoop1(void)
{
	for(int iix=local_ix_min_;iix<=local_ix_max_;iix++){
		int margin = ( local_ix_max_ - local_ix_min_ )/10; 
		bool renew_flag_x = ( iix - local_ix_min_ < margin ) or ( local_ix_max_ - iix < margin );

		for(int iiy=local_iy_min_;iiy<=local_iy_max_;iiy++){
			int margin = ( local_iy_max_ - local_iy_min_ )/10; 
			bool renew_flag = renew_flag_x or ( iiy - local_iy_min_ < margin ) or ( local_iy_max_ - iiy < margin );

			for(int iit=0;iit<cell_num_t_;iit++){
				int i = toIndex(iix, iiy, iit);
				if(states_[i].renew_ and renew_flag){
					states_[i].local_total_cost_ = states_[i].total_cost_;
					states_[i].local_optimal_action_ = states_[i].optimal_action_;
					states_[i].renew_ = false;
				}else
					valueIterationLocal(states_[i]);
			}
		}
	}
}

void ValueIteratorLocal::localValueIterationLoop2(void)
{
	for(int iix=local_ix_max_;iix>=local_ix_min_;iix--){
		int margin = ( local_ix_max_ - local_ix_min_ )/10; 
		bool renew_flag_x = ( iix - local_ix_min_ < margin ) or ( local_ix_max_ - iix < margin );

		for(int iiy=local_iy_max_;iiy>=local_iy_min_;iiy--){
			int margin = ( local_iy_max_ - local_iy_min_ )/10; 
			bool renew_flag = renew_flag_x or ( iiy - local_iy_min_ < margin ) or ( local_iy_max_ - iiy < margin );

			for(int iit=cell_num_t_-1;iit>=0;iit--){
				int i = toIndex(iix, iiy, iit);
				if(states_[i].renew_ and renew_flag){
					states_[i].local_total_cost_ = states_[i].total_cost_;
					states_[i].local_optimal_action_ = states_[i].optimal_action_;
					states_[i].renew_ = false;
				}else
					valueIterationLocal(states_[i]);
			}
		}
	}
}

}
