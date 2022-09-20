#include "value_iteration/ValueIteratorLocal.h"

namespace value_iteration{

ValueIteratorLocal::ValueIteratorLocal(std::vector<Action> &actions, int thread_num) : ValueIterator(actions, thread_num)
{
	local_ix_min_ = local_ix_max_ = local_iy_min_ = local_iy_max_ = 0;
}

void ValueIteratorLocal::setMapWithOccupancyGrid(nav_msgs::OccupancyGrid &map, int theta_cell_num,
		double safety_radius, double safety_radius_penalty,
		double goal_margin_radius, int goal_margin_theta)
{
	ValueIterator::setMapWithOccupancyGrid(map, theta_cell_num, safety_radius, safety_radius_penalty,
			goal_margin_radius, goal_margin_theta);

	local_xy_range_ = 4.0;
	local_ixy_range_ = (int)(local_xy_range_/xy_resolution_);
	local_ix_min_ = 0;
	local_iy_min_ = 0;
	local_ix_max_ = local_ixy_range_*2;
	local_iy_max_ = local_ixy_range_*2;
}

void ValueIteratorLocal::localValueIterationWorker(int id)
{
	while(status_ == "canceled" or status_ == "goal"){
		ROS_INFO("STATUS PROBLEM: %s", status_.c_str());
		status_ = "executing";
	}

	while(status_ != "canceled" and status_ != "goal"){
		chrono::system_clock::time_point start, end;
		start = chrono::system_clock::now();
		localValueIterationLoop();
		end = chrono::system_clock::now();
		double time = static_cast<double>(chrono::duration_cast<chrono::microseconds>(end - start).count() / 1000.0);
		printf("time %lf[ms]\n", time);
	}
}

void ValueIteratorLocal::localValueIterationLoop(void)
{
	for(int iix=local_ix_min_;iix<=local_ix_max_;iix++){
		for(int iiy=local_iy_min_;iiy<=local_iy_max_;iiy++){
			for(int iit=0;iit<cell_num_t_;iit++){
				int i = toIndex(iix, iiy, iit);
				valueIterationLocal(states_[i]);
			}
		}
	}
}

uint64_t ValueIteratorLocal::valueIterationLocal(State &s)
{
	if((not s.free_) or s.final_state_)
		return 0;

	uint64_t min_cost = ValueIterator::max_cost_;
	Action *min_action = NULL;
	for(auto &a : actions_){
		int64_t c = actionCostLocal(s, a);
		if(c < min_cost){
			min_cost = c;
			min_action = &a;
		}
	}

	int64_t delta = min_cost - s.total_cost_;
	s.total_cost_ = min_cost;
	s.optimal_action_ = min_action;

	return delta > 0 ? delta : -delta;
}

Action *ValueIteratorLocal::posToAction(double x, double y, double t_rad)
{
        int ix = (int)floor( (x - map_origin_x_)/xy_resolution_ );
        int iy = (int)floor( (y - map_origin_y_)/xy_resolution_ );

        int t = (int)(180 * t_rad / M_PI);
        int it = (int)floor( ( (t + 360*100)%360 )/t_resolution_ );
	int index = toIndex(ix, iy, it);

	if(states_[index].final_state_){
		status_ = "goal";
		return NULL;
		/*
	}else if(states_[index].local_optimal_action_ != NULL){
		ROS_INFO("COST TO GO: %f", (double)states_[index].local_total_cost_/ValueIterator::prob_base_);
		return states_[index].local_optimal_action_;
		*/
	}else if(states_[index].optimal_action_ != NULL){
		ROS_INFO("COST TO GO: %f", (double)states_[index].total_cost_/ValueIterator::prob_base_);
		return states_[index].optimal_action_;
	}

	return NULL;

}

bool ValueIteratorLocal::inLocalArea(int ix, int iy)
{
	return ix >= local_ix_min_ and ix <= local_ix_max_ and iy >= local_iy_min_ and iy <= local_iy_max_;
}

void ValueIteratorLocal::setLocalCost(const sensor_msgs::LaserScan::ConstPtr &msg, double x, double y, double t)
{
	double start_angle = msg->angle_min;
	for(int i=0; i<msg->ranges.size(); i++){
		double a = t + msg->angle_increment*i + start_angle;

		double lx = x + msg->ranges[i]*cos(a);
		double ly = y + msg->ranges[i]*sin(a);
        	int ix = (int)floor( (lx - map_origin_x_)/xy_resolution_ );
        	int iy = (int)floor( (ly - map_origin_y_)/xy_resolution_ );

		for(double d=0.1;d<=0.9;d+=0.1){
			double half_lx = x + msg->ranges[i]*cos(a)*d;
			double half_ly = y + msg->ranges[i]*sin(a)*d;
	        	int half_ix = (int)floor( (half_lx - map_origin_x_)/xy_resolution_ );
	        	int half_iy = (int)floor( (half_ly - map_origin_y_)/xy_resolution_ );
	
			if(not inLocalArea(half_ix, half_iy))
				continue;
			
			for(int it=0;it<cell_num_t_;it++){
				int index = toIndex(half_ix, half_iy, it);
				states_[index].local_penalty_ /= 2;
			}
		}

		for(int iix=ix-2;iix<=ix+2;iix++){
			for(int iiy=iy-2;iiy<=iy+2;iiy++){

				if(not inLocalArea(iix, iiy))
					continue;

				for(int it=0;it<cell_num_t_;it++){
					int index = toIndex(iix, iiy, it);
					states_[index].local_penalty_ = 2048 << prob_base_bit_;
				}
			}
		}

	}
}

uint64_t ValueIteratorLocal::actionCostLocal(State &s, Action &a)
{
	uint64_t cost = 0;
	for(auto &tran : a._state_transitions[s.it_]){
		int ix = s.ix_ + tran._dix;
		if(ix < 0 or ix >= cell_num_x_)
			return max_cost_;

		int iy = s.iy_ + tran._diy;
		if(iy < 0 or iy >= cell_num_y_)
			return max_cost_;

		int it = (tran._dit + cell_num_t_)%cell_num_t_;

		auto &after_s = states_[toIndex(ix, iy, it)];
		if(not after_s.free_)
			return max_cost_;

		cost += ( after_s.total_cost_ + after_s.penalty_ + after_s.local_penalty_ ) * tran._prob;
	}

	return cost >> prob_base_bit_;
}

void ValueIteratorLocal::setLocalWindow(double x, double y)
{
        int ix = (int)floor( (x - map_origin_x_)/xy_resolution_ );
        int iy = (int)floor( (y - map_origin_y_)/xy_resolution_ );

	local_ix_min_ = ix - local_ixy_range_ >=0 ? ix - local_ixy_range_ : 0;
	local_iy_min_ = iy - local_ixy_range_ >=0 ? iy - local_ixy_range_ : 0;
	local_ix_max_ = ix + local_ixy_range_ < cell_num_x_ ? ix + local_ixy_range_ : cell_num_x_-1;
	local_iy_max_ = iy + local_ixy_range_ < cell_num_y_ ? iy + local_ixy_range_ : cell_num_y_-1;
}

void ValueIteratorLocal::makeLocalValueFunctionMap(nav_msgs::OccupancyGrid &map, int threshold,
		double x, double y, double yaw_rad)
{
	map.header.stamp = ros::Time::now();
	map.header.frame_id = "map";
	map.info.resolution = xy_resolution_;
	map.info.width = local_ixy_range_*2 + 1;
	map.info.height = local_ixy_range_*2 + 1;
	map.info.origin.position.x = x - local_xy_range_;
	map.info.origin.position.y = y - local_xy_range_;
	map.info.origin.orientation = map_origin_quat_;

        int it = (int)floor( ( ((int)(yaw_rad/M_PI*180) + 360*100)%360 )/t_resolution_ );

	for(int y=local_iy_min_; y<=local_iy_max_; y++)
		for(int x=local_ix_min_; x<=local_ix_max_; x++){
			int index = toIndex(x, y, it);
			double cost = (double)states_[index].total_cost_/prob_base_;
			if(cost < (double)threshold)
				map.data.push_back((int)(cost/threshold*250));
			else if(states_[index].free_)
				map.data.push_back(250);
			else 
				map.data.push_back(255);
		}
}

}
