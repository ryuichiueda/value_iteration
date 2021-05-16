#include "value_iteration/ValueIterator.h"
#include <thread>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

namespace value_iteration{

using namespace std;

/* ROSの地図をもらって各セルの情報からStateのオブジェクトを作ってstatesというベクトルに突っ込む */
ValueIterator::ValueIterator(nav_msgs::OccupancyGrid &map, XmlRpc::XmlRpcValue &params,
		int theta_cell_num, int thread_num)
	: cell_num_t_(theta_cell_num), thread_num_(thread_num)
{
	//The cell configurations on XY-plane is set based on the map.
	cell_num_x_ = map.info.width;
	cell_num_y_ = map.info.height;
	ROS_ASSERT(params["theta_cell_num"].getType() == XmlRpc::XmlRpcValue::TypeInt);

	xy_resolution_ = map.info.resolution;
	t_resolution_ = 360/cell_num_t_;

	map_origin_x_ = map.info.origin.position.x;
	map_origin_y_ = map.info.origin.position.y;
	ROS_INFO("ORIGIN: %f, %f", map_origin_x_, map_origin_y_);
	ROS_INFO("MAX: %f, %f", map_origin_x_ + cell_num_x_*xy_resolution_, map_origin_y_ + cell_num_y_*xy_resolution_);

	auto &fs = params["final_state"];
	ROS_ASSERT(fs["width_m"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	goal_x_ = 0.0;//fs["x_center_m"];
	goal_y_ = 0.0;//fs["y_center_m"];
	goal_width_ = fs["width_m"];

	ROS_ASSERT(params["safety_radius"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	setState(map, params["safety_radius"]);

	setAction(params["action_list"]);
	setStateTransition();
}

/* デフォルトのアクションの設定 */
void ValueIterator::setAction(XmlRpc::XmlRpcValue &action_list)
{
	ROS_ASSERT(action_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

	for(int i=0; i<action_list.size(); i++){
		auto &a = action_list[i];
		_actions.push_back(Action(a["name"], a["onestep_forward_m"], a["onestep_rotation_deg"], i));

		auto &b = _actions.back();
		ROS_INFO("set an action: %s, %f, %f", b._name.c_str(), b._delta_fw, b._delta_rot);
	}
}

void ValueIterator::setStateTransition(void)
{
	vector<StateTransition> theta_state_transitions;
	for(auto &a : _actions)
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
	for(auto &a : _actions)
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
	for(auto &a : _actions){
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
	_status.insert(make_pair(id, SweepWorkerStatus()));
	cout << "address:" << &_states[0] << endl;

	for(int j=0; j<times; j++){
		_status[id]._sweep_step = j+1;

		uint64_t max_delta = 0;
	
		int start = rand()%_states.size();
		for(int i=start; i<_states.size(); i++)
			max_delta = max(max_delta, valueIteration(_states[i]));
		for(int i=start-1; i>=0; i--)
			max_delta = max(max_delta, valueIteration(_states[i]));
	
		_status[id]._delta = (double)(max_delta >> prob_base_bit_);
		//delta = max_delta;
		//cout << "delta: " << _status[id]._delta << endl;
		if(_status[id]._delta < 0.1)
			break;
	}

	_status[id]._finished = true;
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

		auto &after_s = _states[toIndex(ix, iy, it)];
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
	while(i<_states.size()){
		ofs << _states[i]._free << " ";
		i += cell_num_t_;
	}

	ofs << flush;
}

void ValueIterator::setState(const nav_msgs::OccupancyGrid &map, double safety_radius)
{
	_states.clear();
	int margin = (int)ceil(safety_radius/xy_resolution_);

	for(int y=0; y<cell_num_y_; y++)
		for(int x=0; x<cell_num_x_; x++)
			for(int t=0; t<cell_num_t_; t++)
				_states.push_back(State(x, y, t, map, margin, cell_num_x_));
}

void ValueIterator::setStateValues(void)
{
	for(auto &s : _states){
		double x0 = s._ix*xy_resolution_ + map_origin_x_;
		double y0 = s._iy*xy_resolution_ + map_origin_y_;
		double x1 = x0 + xy_resolution_;
		double y1 = y0 + xy_resolution_;

		s._final_state = fabs(x0 - goal_x_) < goal_width_
			       && fabs(y0 - goal_y_) < goal_width_
			       && fabs(x1 - goal_x_) < goal_width_ 
			       && fabs(y1 - goal_y_) < goal_width_
			       && s._free;
	}

	for(auto &s : _states){
		if(s._final_state)
			s._cost = 0;
		else
			s._cost = max_cost_;
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
		while(i<_states.size()){
			auto &s = _states[i];
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
		while(i<_states.size()){
			uint64_t v = _states[i]._cost*5 >> prob_base_bit_;
			if(_states[i]._free and v <= 255)
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
		while(i<_states.size()){
			auto &s = _states[i];
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
		while(i<_states.size()){

			if(_states[i]._optimal_action == NULL){
				action_file << "0 0 0" << endl;
			}else if(_states[i]._optimal_action->_name == "forward"){
				action_file << "0 255 0" << endl;
			}else if(_states[i]._optimal_action->_name == "left"){
				action_file << "0 0 255" << endl;
			}else if(_states[i]._optimal_action->_name == "right"){
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

	ROS_INFO("VALUE: %f", (double)_states[index]._cost/ValueIterator::prob_base_);

	if(_states[index]._optimal_action != NULL)
		ROS_INFO("CMDVEL: %f, %f", _states[index]._optimal_action->_delta_fw, 
			_states[index]._optimal_action->_delta_rot);

	return _states[index]._optimal_action;
}

void ValueIterator::setGoal(double goal_x, double goal_y)
{
	goal_x_ = goal_x;
	goal_y_ = goal_y;

	_status.clear();
	setStateValues();
}

}
