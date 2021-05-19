#include "value_iteration/ValueIterator.h"
#include <thread>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

namespace value_iteration{

using namespace std;

/* ROSの地図をもらって各セルの情報からStateのオブジェクトを作ってstatesというベクトルに突っ込む */
ValueIterator::ValueIterator(nav_msgs::OccupancyGrid &map, 
		vector<Action> &actions, int theta_cell_num, int thread_num,
		double safety_radius, double goal_margin_radius, int goal_margin_theta)
	: actions_(actions), cell_num_t_(theta_cell_num), thread_num_(thread_num), 
	  goal_x_(0.0), goal_y_(0.0), goal_t_(0), 
	  goal_margin_radius_(goal_margin_radius), goal_margin_theta_(goal_margin_theta)
{
	cell_num_x_ = map.info.width;
	cell_num_y_ = map.info.height;

	xy_resolution_ = map.info.resolution;
	t_resolution_ = 360/cell_num_t_;

	map_origin_x_ = map.info.origin.position.x;
	map_origin_y_ = map.info.origin.position.y;

	setState(map, safety_radius);
	setStateTransition();
}

bool ValueIterator::finished(std_msgs::UInt32MultiArray &sweep_times, std_msgs::Float32MultiArray &deltas)
{ 
	sweep_times.data.resize(thread_num_);
	deltas.data.resize(thread_num_);

	bool finish = true;
	for(int t=0; t<thread_num_; t++){
		sweep_times.data[t] = status_[t]._sweep_step;
		deltas.data[t] = status_[t]._delta;
		finish &= status_[t]._finished;
	}
	return finish;
}

void ValueIterator::setStateTransition(void)
{
	vector<StateTransition> theta_state_transitions;
	for(auto &a : actions_)
		for(int t=0; t<cell_num_t_; t++)
			a._state_transitions.push_back(theta_state_transitions);

	vector<thread> ths;
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

void ValueIterator::accurateStateTransition(Action &a, 
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
				accurateStateTransition(a, ox, oy, ot + theta_origin, dx, dy, dt);
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
	if((not s._free) or s._final_state)
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

	int64_t delta = min_cost - s._cost;
	s._cost = min_cost;
	s._optimal_action = min_action;

	return delta > 0 ? delta : -delta;
}

void ValueIterator::valueIterationWorker(int times, int id)
{
	status_.insert(make_pair(id, SweepWorkerStatus()));
	cout << "address:" << &states_[0] << endl;

	for(int j=0; j<times; j++){
		status_[id]._sweep_step = j+1;

		uint64_t max_delta = 0;
	
		int start = rand()%states_.size();
		for(int i=start; i<states_.size(); i++)
			max_delta = max(max_delta, valueIteration(states_[i]));
		for(int i=start-1; i>=0; i--)
			max_delta = max(max_delta, valueIteration(states_[i]));
	
		status_[id]._delta = (double)(max_delta >> prob_base_bit_);
		//delta = max_delta;
		//cout << "delta: " << status_[id]._delta << endl;
		if(status_[id]._delta < 0.1)
			break;
	}

	status_[id]._finished = true;
}

int ValueIterator::toIndex(int ix, int iy, int it)
{
	return it + ix*cell_num_t_ + iy*(cell_num_t_*cell_num_x_);
}

uint64_t ValueIterator::actionCost(State &s, Action &a)
{
	uint64_t cost = 0;
	for(auto &tran : a._state_transitions[s._it]){
		int ix = s._ix + tran._dix;
		if(ix < 0 or ix >= cell_num_x_)
			return max_cost_;

		int iy = s._iy + tran._diy;
		if(iy < 0 or iy >= cell_num_y_)
			return max_cost_;

		//ROS_INFO("ANGLE: %d, %d", s._it, tran._dit);
		int it = (/*s._it +*/ tran._dit + cell_num_t_)%cell_num_t_;

		auto &after_s = states_[toIndex(ix, iy, it)];
		if(not after_s._free)
			return max_cost_;

		cost += (after_s._cost>>prob_base_bit_) * tran._prob;
	}

	return cost + prob_base_;
}

/* statesのセルの情報をPBMとして出力（デバッグ用） */
void ValueIterator::outputPbmMap(void)
{
	ofstream ofs("/tmp/a.pbm");

	ofs << "P1" << endl;
	ofs << cell_num_x_ << " " << cell_num_y_ << endl;
	int i = 0;
	while(i<states_.size()){
		ofs << states_[i]._free << " ";
		i += cell_num_t_;
	}

	ofs << flush;
}

void ValueIterator::setState(const nav_msgs::OccupancyGrid &map, double safety_radius)
{
	states_.clear();
	int margin = (int)ceil(safety_radius/xy_resolution_);

	for(int y=0; y<cell_num_y_; y++)
		for(int x=0; x<cell_num_x_; x++)
			for(int t=0; t<cell_num_t_; t++)
				states_.push_back(State(x, y, t, map, margin, cell_num_x_));
}

void ValueIterator::setStateValues(void)
{
	for(auto &s : states_){
		/* goal distance check */
		double x0 = s._ix*xy_resolution_ + map_origin_x_;
		double y0 = s._iy*xy_resolution_ + map_origin_y_;
		double r0 = (x0 - goal_x_)*(x0 - goal_x_) + (y0 - goal_y_)*(y0 - goal_y_);

		double x1 = x0 + xy_resolution_;
		double y1 = y0 + xy_resolution_;
		double r1 = (x1 - goal_x_)*(x1 - goal_x_) + (y1 - goal_y_)*(y1 - goal_y_);

		s._final_state = r0 < goal_margin_radius_*goal_margin_radius_ 
			       && r1 < goal_margin_radius_*goal_margin_radius_
			       && s._free;

		/* orientation check */
		int t0 = s._it*t_resolution_;
		int t1 = (s._it+1)*t_resolution_;
		int goal_t_2 = goal_t_ > 180 ? goal_t_ - 360 : goal_t_ + 360;

		s._final_state &= 
			(goal_t_ - goal_margin_theta_ <= t0 and t1 <= goal_t_ + goal_margin_theta_) or 
			(goal_t_2 - goal_margin_theta_ <= t0 and t1 <= goal_t_2 + goal_margin_theta_);
	}

	for(auto &s : states_){
		s._cost = s._final_state ? 0 : max_cost_;
		/*
		if(s._final_state)
			s._cost = 0;
		else
			s._cost = max_cost_;
			*/
	}
}

bool ValueIterator::outputValuePgmMap(grid_map_msgs::GetGridMap::Response& response)
{
	grid_map::GridMap map;
	map.setFrameId("map");
	map.setGeometry(grid_map::Length(cell_num_x_*xy_resolution_, cell_num_y_*xy_resolution_), xy_resolution_);

	for(int t=0; t<cell_num_t_; t++){
		string name = to_string(t);

		map.add(name);
		int i = t;
		while(i<states_.size()){
			auto &s = states_[i];
			map.at(name, grid_map::Index(s._ix, s._iy)) = s._cost/(ValueIterator::prob_base_);

			i += cell_num_t_;
		}
	}

	grid_map_msgs::GridMap message;
	grid_map::GridMapRosConverter::toMessage(map, message);
	response.map = message;

	for(int t=0; t<cell_num_t_; t++){
		ofstream ofs("/tmp/value_t=" + to_string(t) + ".pgm");

		ofs << "P2" << endl;
		ofs << cell_num_x_ << " " << cell_num_y_ << " 255" << endl;
		int i = t;
		while(i<states_.size()){
			uint64_t v = states_[i]._cost*5 >> prob_base_bit_;
			if(states_[i]._free and v <= 255)
				ofs << 255 - v << '\n';
			else
				ofs << 0 << '\n';

			i += cell_num_t_;
		}
		ofs << flush;
	}
	return true;
}

bool ValueIterator::actionImageWriter(grid_map_msgs::GetGridMap::Response& response)
{
	grid_map::GridMap map;
	map.setFrameId("map");
	map.setGeometry(grid_map::Length(cell_num_x_*xy_resolution_, cell_num_y_*xy_resolution_), xy_resolution_);

	for(int t=0; t<cell_num_t_; t++){
		string name = to_string(t);

		map.add(name);
		int i = t;
		while(i<states_.size()){
			auto &s = states_[i];
			if(s._optimal_action == NULL){
				map.at(name, grid_map::Index(s._ix, s._iy)) = -1.0;
			}else{
				map.at(name, grid_map::Index(s._ix, s._iy)) = (double)s._optimal_action->id_;
			}

			i += cell_num_t_;
		}
	}

	grid_map_msgs::GridMap message;
	grid_map::GridMapRosConverter::toMessage(map, message);
	response.map = message;

	for(int t=0; t<cell_num_t_; t++){
		ofstream action_file("/tmp/action_t=" + to_string(t) + ".ppm");

		action_file << "P3" << endl;
		action_file << cell_num_x_ << " " << cell_num_y_ << " 255" << endl;
		int i = t;
		while(i<states_.size()){

			if(states_[i]._optimal_action == NULL){
				action_file << "0 0 0" << endl;
			}else if(states_[i]._optimal_action->_name == "forward"){
				action_file << "0 255 0" << endl;
			}else if(states_[i]._optimal_action->_name == "left"){
				action_file << "0 0 255" << endl;
			}else if(states_[i]._optimal_action->_name == "right"){
				action_file << "255 0 0" << endl;
			}
			i += cell_num_t_;
		}

		action_file << flush;
	}

	return true;
}


Action *ValueIterator::posToAction(double x, double y, double t_rad)
{
        int ix = (int)floor( (x - map_origin_x_)/xy_resolution_ );
        int iy = (int)floor( (y - map_origin_y_)/xy_resolution_ );

        int t = (int)(180 * t_rad / M_PI);
        int it = (int)floor( ( (t + 360*100)%360 )/t_resolution_ );
	ROS_INFO("CELL: %d, %d, %d", ix, iy, it);
	int index = toIndex(ix, iy, it);

	ROS_INFO("VALUE: %f", (double)states_[index]._cost/ValueIterator::prob_base_);

/*	if(states_[index]._optimal_action != NULL)
		ROS_INFO("CMDVEL: %f, %f", states_[index]._optimal_action->_delta_fw, 
			states_[index]._optimal_action->_delta_rot); */

	if(states_[index]._final_state)
		return NULL;

	return states_[index]._optimal_action;
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

	status_.clear();
	setStateValues();
}

}
