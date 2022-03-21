#include "value_iteration/ValueIterator.h"
#include <thread>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

namespace value_iteration{

ValueIterator::ValueIterator(std::vector<Action> &actions, int thread_num)
	: actions_(actions), thread_num_(thread_num), status_("init"), 
	  goal_x_(0.0), goal_y_(0.0), goal_t_(0)
{
}

void ValueIterator::setMapWithOccupancyGrid(nav_msgs::OccupancyGrid &map, int theta_cell_num,
		double safety_radius, double safety_radius_penalty,
		double goal_margin_radius, int goal_margin_theta)
{
	cell_num_t_ = theta_cell_num;
	goal_margin_radius_ = goal_margin_radius;
	goal_margin_theta_ = goal_margin_theta;

	cell_num_x_ = map.info.width;
	cell_num_y_ = map.info.height;

	xy_resolution_ = map.info.resolution;
	t_resolution_ = 360/cell_num_t_;

	map_origin_x_ = map.info.origin.position.x;
	map_origin_y_ = map.info.origin.position.y;
	map_origin_quat_ = map.info.origin.orientation;

	ROS_INFO("SET STATES START");
	setState(map, safety_radius, safety_radius_penalty);
	setStateTransition();
	setSweepOrders();
	ROS_INFO("SET STATES END");
}

void ValueIterator::setMapWithCostGrid(nav_msgs::OccupancyGrid &map, int theta_cell_num,
		double safety_radius, double safety_radius_penalty,
		double goal_margin_radius, int goal_margin_theta)
{
	cell_num_t_ = theta_cell_num;
	goal_margin_radius_ = goal_margin_radius;
	goal_margin_theta_ = goal_margin_theta;

	cell_num_x_ = map.info.width;
	cell_num_y_ = map.info.height;

	xy_resolution_ = map.info.resolution;
	t_resolution_ = 360/cell_num_t_;

	map_origin_x_ = map.info.origin.position.x;
	map_origin_y_ = map.info.origin.position.y;
	map_origin_quat_ = map.info.origin.orientation;

	states_.clear();
	int margin = (int)ceil(safety_radius/xy_resolution_);

	for(int y=0; y<cell_num_y_; y++)
		for(int x=0; x<cell_num_x_; x++){
			unsigned int cost = (unsigned int)(map.data[x + cell_num_x_*y] & 0xFF);
			for(int t=0; t<cell_num_t_; t++)
				states_.push_back(State(x, y, t, cost));
		}

	setStateTransition();
	setSweepOrders();
}

bool ValueIterator::finished(std_msgs::UInt32MultiArray &sweep_times, std_msgs::Float32MultiArray &deltas)
{ 
	sweep_times.data.resize(thread_num_);
	deltas.data.resize(thread_num_);

	bool finish = true;
	for(int t=0; t<thread_num_; t++){
		sweep_times.data[t] = thread_status_[t]._sweep_step;
		deltas.data[t] = thread_status_[t]._delta;
		finish &= thread_status_[t]._finished;
	}
	return finish;
}

void ValueIterator::setStateTransition(void)
{
	std::vector<StateTransition> theta_state_transitions;
	for(auto &a : actions_)
		for(int t=0; t<cell_num_t_; t++)
			a._state_transitions.push_back(theta_state_transitions);

	std::vector<thread> ths;
	for(int t=0; t<cell_num_t_; t++)
		ths.push_back(thread(&ValueIterator::setStateTransitionWorker, this, t));

	for(auto &th : ths)
		th.join();
}

void ValueIterator::cellDelta(double x, double y, double t, int &ix, int &iy, int &it)
{
	ix = (int)floor(fabs(x) / xy_resolution_);
	if(x < 0.0)
		ix = -ix-1;
	iy = (int)floor(fabs(y) / xy_resolution_);
	if(y < 0.0)
		iy = -iy-1;

	it = (int)floor(t / t_resolution_);
}


void ValueIterator::setStateTransitionWorker(int it)
{
	for(auto &a : actions_)
		setStateTransitionWorkerSub(a, it);
}

void ValueIterator::noNoiseStateTransition(Action &a, 
	double from_x, double from_y, double from_t, double &to_x, double &to_y, double &to_t)
{
	double ang = from_t / 180 * M_PI;
	to_x = from_x + a._delta_fw*cos(ang);
	to_y = from_y + a._delta_fw*sin(ang);
	to_t = from_t + a._delta_rot;
	while(to_t < 0.0)
		to_t += 360.0;
}

void ValueIterator::setStateTransitionWorkerSub(Action &a, int it)
{
	double theta_origin = it*t_resolution_;
	const int xy_sample_num = 1<<ValueIterator::resolution_xy_bit_;
	const int t_sample_num = 1<<ValueIterator::resolution_t_bit_;
	const double xy_step = xy_resolution_/xy_sample_num;
	const double t_step = t_resolution_/t_sample_num;

	for(double oy=0.5*xy_step; oy<xy_resolution_; oy+=xy_step){
		for(double ox=0.5*xy_step; ox<xy_resolution_; ox+=xy_step){
			for(double ot=0.5*t_step; ot<t_resolution_; ot+=t_step){

				//遷移後の姿勢
				double dx, dy, dt;
				noNoiseStateTransition(a, ox, oy, ot + theta_origin, dx, dy, dt);
				int dix, diy, dit;
				cellDelta(dx, dy, dt, dix, diy, dit); 

				bool exist = false;
				for(auto &s : a._state_transitions[it]){
					if(s._dix == dix and s._diy == diy and s._dit == dit){
						s._prob++;
						exist = true;
						break;
					}
				}
				if(not exist)
					a._state_transitions[it].push_back(StateTransition(dix, diy, dit, 1));
			}
		}
	}
}

uint64_t ValueIterator::valueIteration(State &s)
{
	if((not s.free_) or s.final_state_)
		return 0;

	uint64_t min_cost = ValueIterator::max_cost_;
	Action *min_action = NULL;
	for(auto &a : actions_){
		int64_t c = actionCost(s, a);
		if(c < min_cost){
			min_cost = c;
			min_action = &a;
		}
	}

	int64_t delta = min_cost - s.total_cost_;
	s.total_cost_ = min_cost;
	s.optimal_action_ = min_action;
	//s.renew_ = true;

	return delta > 0 ? delta : -delta;
}

void ValueIterator::valueIterationWorker(int times, int id)
{
	thread_status_.insert(make_pair(id, SweepWorkerStatus()));

	for(int j=0; j<times; j++){
		thread_status_[id]._sweep_step = j+1;

		uint64_t max_delta = 0;
		for(auto i : sweep_orders_[id%sweep_orders_.size()])
			max_delta = max(max_delta, valueIteration(states_[i]));
	
		thread_status_[id]._delta = (double)(max_delta >> prob_base_bit_);
		if(thread_status_[id]._delta < 0.1 or status_ == "canceled" or status_ == "goal" )
			break;
	}

	thread_status_[id]._finished = true;
}

int ValueIterator::toIndex(int ix, int iy, int it)
{
	return it + ix*cell_num_t_ + iy*(cell_num_t_*cell_num_x_);
}

bool ValueIterator::inMapArea(int ix, int iy)
{
	return ix >= 0 and ix < cell_num_x_ and iy >= 0 and iy < cell_num_y_;
}

uint64_t ValueIterator::actionCost(State &s, Action &a)
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

void ValueIterator::setState(const nav_msgs::OccupancyGrid &map, double safety_radius, double safety_radius_penalty)
{
	states_.clear();
	int margin = (int)ceil(safety_radius/xy_resolution_);

	for(int y=0; y<cell_num_y_; y++)
		for(int x=0; x<cell_num_x_; x++)
			for(int t=0; t<cell_num_t_; t++)
				states_.push_back(State(x, y, t, map, margin, safety_radius_penalty, cell_num_x_));
}

void ValueIterator::setStateValues(void)
{
	for(auto &s : states_){
		/* goal distance check */
		double x0 = s.ix_*xy_resolution_ + map_origin_x_;
		double y0 = s.iy_*xy_resolution_ + map_origin_y_;
		double r0 = (x0 - goal_x_)*(x0 - goal_x_) + (y0 - goal_y_)*(y0 - goal_y_);

		double x1 = x0 + xy_resolution_;
		double y1 = y0 + xy_resolution_;
		double r1 = (x1 - goal_x_)*(x1 - goal_x_) + (y1 - goal_y_)*(y1 - goal_y_);

		s.final_state_ = r0 < goal_margin_radius_*goal_margin_radius_ 
			       && r1 < goal_margin_radius_*goal_margin_radius_
			       && s.free_;

		/* orientation check */
		int t0 = s.it_*t_resolution_;
		int t1 = (s.it_+1)*t_resolution_;
		int goal_t_2 = goal_t_ > 180 ? goal_t_ - 360 : goal_t_ + 360;

		s.final_state_ &= 
			(goal_t_ - goal_margin_theta_ <= t0 and t1 <= goal_t_ + goal_margin_theta_) or 
			(goal_t_2 - goal_margin_theta_ <= t0 and t1 <= goal_t_2 + goal_margin_theta_);
	}

	for(auto &s : states_){
		//s.renew_ = false;
		s.total_cost_ = s.final_state_ ? 0 : max_cost_;
		//s.local_total_cost_ = s.total_cost_;
		s.local_penalty_ = 0;
		s.optimal_action_ = NULL;
		//s.local_optimal_action_ = NULL;
	}
}

bool ValueIterator::valueFunctionWriter(grid_map_msgs::GetGridMap::Response& response)
{
	grid_map::GridMap map;
	map.setFrameId("map");
	map.setGeometry(grid_map::Length(cell_num_x_*xy_resolution_, cell_num_y_*xy_resolution_), xy_resolution_);

	for(int t=0; t<cell_num_t_; t++){
		std::string name = to_string(t);

		map.add(name);
		for(int i=t; i<states_.size(); i+=cell_num_t_){
			auto &s = states_[i];
			map.at(name, grid_map::Index(s.ix_, s.iy_)) = s.total_cost_/(ValueIterator::prob_base_);
		}
	}

	grid_map_msgs::GridMap message;
	grid_map::GridMapRosConverter::toMessage(map, message);
	response.map = message;

	return true;
}

bool ValueIterator::policyWriter(grid_map_msgs::GetGridMap::Response& response)
{
	grid_map::GridMap map;
	map.setFrameId("map");
	map.setGeometry(grid_map::Length(cell_num_x_*xy_resolution_, cell_num_y_*xy_resolution_), xy_resolution_);

	for(int t=0; t<cell_num_t_; t++){
		std::string name = to_string(t);

		map.add(name);
		for(int i=t; i<states_.size(); i+=cell_num_t_){
			auto &s = states_[i];
			map.at(name, grid_map::Index(s.ix_, s.iy_)) = 
				(s.optimal_action_ == NULL) ? -1.0 : (double)s.optimal_action_->id_;
		}
	}

	grid_map_msgs::GridMap message;
	grid_map::GridMapRosConverter::toMessage(map, message);
	response.map = message;

	return true;
}


void ValueIterator::setGoal(double goal_x, double goal_y, int goal_t)
{
	while(goal_t < 0)
		goal_t += 360;
	while(goal_t >= 360)
		goal_t -= 360;

	goal_x_ = goal_x;
	goal_y_ = goal_y;
	goal_t_ = goal_t;

	ROS_INFO("GOAL: %f, %f, %d", goal_x_, goal_y_, goal_t_);

	thread_status_.clear();
	setStateValues();
	status_ = "calculating";
}

void ValueIterator::makeValueFunctionMap(nav_msgs::OccupancyGrid &map, int threshold, 
		double x, double y, double yaw_rad)
{
	map.header.stamp = ros::Time::now();
	map.header.frame_id = "map";
	map.info.resolution = xy_resolution_;
	map.info.width = cell_num_x_;
	map.info.height = cell_num_y_;
	map.info.origin.position.x = map_origin_x_;
	map.info.origin.position.y = map_origin_y_;
	map.info.origin.orientation = map_origin_quat_;

        int it = (int)floor( ( ((int)(yaw_rad/M_PI*180) + 360*100)%360 )/t_resolution_ );

	for(int y=0; y<cell_num_y_; y++)
		for(int x=0; x<cell_num_x_; x++){
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


void ValueIterator::setSweepOrders(void)
{
	if(sweep_orders_.size())
		return;

	ROS_INFO("SET SWEEP ORDER");
	sweep_orders_.push_back( std::vector<int>() );
	for(int y=0; y<cell_num_y_; y++)
		for(int x=0; x<cell_num_x_; x++)
			for(int t=0; t<cell_num_t_; t++)
				sweep_orders_[0].push_back(toIndex(x,y,t));

	sweep_orders_.push_back( std::vector<int>() );
	for(int x=0; x<cell_num_x_; x++)
		for(int y=0; y<cell_num_y_; y++)
			for(int t=0; t<cell_num_t_; t++)
				sweep_orders_[1].push_back(toIndex(x,y,t));

	sweep_orders_.push_back( {sweep_orders_[0].rbegin(), sweep_orders_[0].rend()} );//2
	sweep_orders_.push_back( {sweep_orders_[1].rbegin(), sweep_orders_[1].rend()} );//3

	/* 4,5 */
	int half = sweep_orders_[0].size()/2;
	for(int i=0;i<2;i++){
		sweep_orders_.push_back( {sweep_orders_[i].begin(), sweep_orders_[i].begin()+half} );
		sweep_orders_[4].insert(sweep_orders_[4].end(),
			sweep_orders_[i].begin()+half, sweep_orders_[i].end() );
	}
}

void ValueIterator::setCancel(void)
{
	status_ = "canceled";
}

bool ValueIterator::endOfTrial(void)
{
	return status_ == "canceled" or status_ == "goal";
}

bool ValueIterator::arrived(void)
{
	return status_ == "goal";
}

Action *ValueIterator::posToAction(double x, double y, double t_rad)
{
        int ix = (int)floor( (x - map_origin_x_)/xy_resolution_ );
        int iy = (int)floor( (y - map_origin_y_)/xy_resolution_ );

        int t = (int)(180 * t_rad / M_PI);
        int it = (int)floor( ( (t + 360*100)%360 )/t_resolution_ );
	int index = toIndex(ix, iy, it);

	if(states_[index].final_state_){
		status_ = "goal";
		return NULL;
	}else if(states_[index].optimal_action_ != NULL){
		ROS_INFO("COST TO GO: %f", (double)states_[index].total_cost_/ValueIterator::prob_base_);
		return states_[index].optimal_action_;
	}

	return NULL;

}

void ValueIterator::setCalculated(void)
{
	if(status_ != "canceled")
		status_ = "calculated";
}

bool ValueIterator::isCalculated(void)
{
	return status_ == "calculated";
}

}
