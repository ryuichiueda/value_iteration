#include "value_iteration/ValueIterator.h"
#include <thread>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

namespace value_iteration{

using namespace std;

/* ROSの地図をもらって各セルの情報からStateのオブジェクトを作ってstatesというベクトルに突っ込む */
ValueIterator::ValueIterator(nav_msgs::OccupancyGrid &map, XmlRpc::XmlRpcValue &params)
{
	//The cell configurations on XY-plane is set based on the map.
	_cell_x_num = map.info.width;
	_cell_y_num = map.info.height;
	ROS_ASSERT(params["theta_cell_num"].getType() == XmlRpc::XmlRpcValue::TypeInt);
	_cell_t_num = params["theta_cell_num"];

	_cell_x_width = map.info.resolution;
	_cell_y_width = map.info.resolution;
	_cell_t_width = 360/_cell_t_num;

	map_origin_x_ = map.info.origin.position.x;
	map_origin_y_ = map.info.origin.position.y;
	ROS_INFO("ORIGIN: %f, %f", map_origin_x_, map_origin_y_);
	ROS_INFO("MAX: %f, %f", map_origin_x_ + _cell_x_num*_cell_x_width, map_origin_y_ + _cell_y_num*_cell_y_width);

	auto &fs = params["final_state"];
	ROS_ASSERT(fs["width_m"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	_final_state_x = 0.0;//fs["x_center_m"];
	_final_state_y = 0.0;//fs["y_center_m"];
	_final_state_width = fs["width_m"];

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
		for(int t=0; t<_cell_t_num; t++)
			a._state_transitions.push_back(theta_state_transitions);

	vector<thread> ths;
	for(int t=0; t<_cell_t_num; t++)
		ths.push_back(thread(&ValueIterator::setStateTransitionWorker, this, t));

	for(auto &th : ths)
		th.join();
}

void ValueIterator::toCellPos(double x, double y, double t, int &ix, int &iy, int &it)
{
	ix = (int)floor(fabs(x) / _cell_x_width);
	if(x < 0.0)
		ix = -ix-1;
	iy = (int)floor(fabs(y) / _cell_y_width);
	if(y < 0.0)
		iy = -iy-1;

	it = (int)floor(t / _cell_t_width);
}


void ValueIterator::setStateTransitionWorker(int it)
{
	for(auto &a : _actions)
		setStateTransition(a, it);
}

void ValueIterator::accurateStateTransition(Action &a, double from_x, double from_y, double from_t, double &to_x, double &to_y, double &to_t)
{
	double ang = from_t / 180 * 3.141592;
	to_x = from_x + a._delta_fw*cos(ang);
	to_y = from_y + a._delta_fw*sin(ang);
	to_t = from_t + a._delta_rot;
	while(to_t < 0.0)
		to_t += 360.0;
}

void ValueIterator::setStateTransition(Action &a, int it)
{
	double theta_origin = it*_cell_t_width;

	for(int y=0; y<_resolution_y; y++){
		for(int x=0; x<_resolution_x; x++){
			for(int t=0; t<_resolution_t; t++){
				//遷移前の姿勢
				double ox = ((double)x+0.5)*_cell_x_width/_resolution_x;
				double oy = ((double)y+0.5)*_cell_y_width/_resolution_y;
				double ot = ((double)t+0.5)*_cell_t_width/_resolution_t + theta_origin;

				//遷移後の姿勢
				double dx, dy, dt;
				accurateStateTransition(a, ox, oy, ot, dx, dy, dt);
				int dix, diy, dit;
				toCellPos(dx, dy, dt, dix, diy, dit); 

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

	uint64_t min_cost = ValueIterator::_max_cost;
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
	
		_status[id]._delta = (double)(max_delta >> _prob_base_bit);
		//delta = max_delta;
		//cout << "delta: " << _status[id]._delta << endl;
		if(_status[id]._delta < 0.1)
			break;
	}

	_status[id]._finished = true;
}

int ValueIterator::toIndex(int ix, int iy, int it)
{
	return it + ix*_cell_t_num + iy*(_cell_t_num*_cell_x_num);
}

uint64_t ValueIterator::actionCost(State &s, Action &a)
{
	uint64_t cost = 0;
	for(auto &tran : a._state_transitions[s._it]){
		int ix = s._ix + tran._dix;
		if(ix < 0 or ix >= _cell_x_num)
			return _max_cost;

		int iy = s._iy + tran._diy;
		if(iy < 0 or iy >= _cell_y_num)
			return _max_cost;

		//ROS_INFO("ANGLE: %d, %d", s._it, tran._dit);
		int it = (/*s._it +*/ tran._dit + _cell_t_num)%_cell_t_num;

		auto &after_s = _states[toIndex(ix, iy, it)];
		if(not after_s._free)
			return _max_cost;

		cost += (after_s._cost>>_prob_base_bit) * tran._prob;
	}

	return cost + _prob_base;
}

/* statesのセルの情報をPBMとして出力（デバッグ用） */
void ValueIterator::outputPbmMap(void)
{
	ofstream ofs("/tmp/a.pbm");

	ofs << "P1" << endl;
	ofs << _cell_x_num << " " << _cell_y_num << endl;
	int i = 0;
	while(i<_states.size()){
		ofs << _states[i]._free << " ";
		i += _cell_t_num;
	}

	ofs << flush;
}

void ValueIterator::setState(const nav_msgs::OccupancyGrid &map, double safety_radius)
{
	_states.clear();
	int margin = (int)ceil(safety_radius/_cell_x_width);

	for(int y=0; y<_cell_y_num; y++)
		for(int x=0; x<_cell_x_num; x++)
			for(int t=0; t<_cell_t_num; t++)
				_states.push_back(State(x, y, t, map, margin, _cell_x_num));
}

void ValueIterator::setStateValues(void)
{
	for(auto &s : _states){
		/*
		double x0 = (s._ix - _center_state_ix)*_cell_x_width;
		double y0 = (s._iy - _center_state_iy)*_cell_y_width;
		*/
		double x0 = s._ix*_cell_x_width + map_origin_x_;
		double y0 = s._iy*_cell_y_width + map_origin_y_;
		double x1 = x0 + _cell_x_width;
		double y1 = y0 + _cell_y_width;

		s._final_state = fabs(x0 - _final_state_x) < _final_state_width
			       && fabs(y0 - _final_state_y) < _final_state_width
			       && fabs(x1 - _final_state_x) < _final_state_width 
			       && fabs(y1 - _final_state_y) < _final_state_width
			       && s._free;
	}

	for(auto &s : _states){
		if(s._final_state)
			s._cost = 0;
		else
			s._cost = _max_cost;
	}
}

bool ValueIterator::outputValuePgmMap(grid_map_msgs::GetGridMap::Response& response)
{
	grid_map::GridMap map;
	map.setFrameId("map");
	map.setGeometry(grid_map::Length(_cell_x_num*_cell_x_width, _cell_y_num*_cell_y_width), _cell_x_width);

	for(int t=0; t<_cell_t_num; t++){
		string name = to_string(t);

		map.add(name);
		int i = t;
		while(i<_states.size()){
			auto &s = _states[i];
			map.at(name, grid_map::Index(s._ix, s._iy)) = s._cost/(ValueIterator::_prob_base);

			i += _cell_t_num;
		}
	}

	grid_map_msgs::GridMap message;
	grid_map::GridMapRosConverter::toMessage(map, message);
	response.map = message;

	for(int t=0; t<_cell_t_num; t++){
		ofstream ofs("/tmp/value_t=" + to_string(t) + ".pgm");

		ofs << "P2" << endl;
		ofs << _cell_x_num << " " << _cell_y_num << " 255" << endl;
		int i = t;
		while(i<_states.size()){
			uint64_t v = _states[i]._cost*5 >> _prob_base_bit;
			if(_states[i]._free and v <= 255)
				ofs << 255 - v << '\n';
			else
				ofs << 0 << '\n';

			i += _cell_t_num;
		}
		ofs << flush;
	}
	return true;
}

bool ValueIterator::actionImageWriter(grid_map_msgs::GetGridMap::Response& response)
{
	grid_map::GridMap map;
	map.setFrameId("map");
	map.setGeometry(grid_map::Length(_cell_x_num*_cell_x_width, _cell_y_num*_cell_y_width), _cell_x_width);

	for(int t=0; t<_cell_t_num; t++){
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

			i += _cell_t_num;
		}
	}

	grid_map_msgs::GridMap message;
	grid_map::GridMapRosConverter::toMessage(map, message);
	response.map = message;

	for(int t=0; t<_cell_t_num; t++){
		ofstream action_file("/tmp/action_t=" + to_string(t) + ".ppm");

		action_file << "P3" << endl;
		action_file << _cell_x_num << " " << _cell_y_num << " 255" << endl;
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
			i += _cell_t_num;
		}

		action_file << flush;
	}

	return true;
}


Action *ValueIterator::posToAction(double x, double y, double t_rad)
{
        int ix = (int)floor( (x - map_origin_x_)/_cell_x_width );
        int iy = (int)floor( (y - map_origin_y_)/_cell_y_width );

        int t = (int)(180 * t_rad / M_PI);
        int it = (int)floor( ( (t + 360*100)%360 )/_cell_t_width );
	ROS_INFO("CELL: %d, %d, %d", ix, iy, it);
	int index = toIndex(ix, iy, it);

	ROS_INFO("VALUE: %f", (double)_states[index]._cost/ValueIterator::_prob_base);

	if(_states[index]._optimal_action != NULL)
		ROS_INFO("CMDVEL: %f, %f", _states[index]._optimal_action->_delta_fw, 
			_states[index]._optimal_action->_delta_rot);

	return _states[index]._optimal_action;
}

void ValueIterator::initialize(double goal_x, double goal_y)
{
	_final_state_x = goal_x;
	_final_state_y = goal_y;

	_status.clear();
	setStateValues();
}

}
