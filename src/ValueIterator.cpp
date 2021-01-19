#include "ValueIterator.h"
#include <thread>
using namespace std;

State::State(int x, int y, int theta, int map_value)
{
	_ix = x;
	_iy = y;
	_it = theta;
	_value = ValueIterator::_value_min;
	_free = (map_value == 0);
	_final_state = false;
}

Action::Action(string name, double fw, double rot)
{
	_name = name;

	_delta_fw = fw;
	_delta_rot = rot;

	//_delta_fw_stdev = fabs(fw)*0.1;
	//_delta_rot_stdev = fabs(rot)*0.1;
}

StateTransition::StateTransition(int dix, int diy, int dit, double prob)
{
	_dix = dix;
	_diy = diy;
	_dit = dit;
	_prob = prob;
}

string StateTransition::to_string(void)
{
	return "dix:" + std::to_string(_dix) + " diy:" + std::to_string(_diy) 
		+ " dit:" + std::to_string(_dit) + " prob:" + std::to_string(_prob);
}

/* ROSの地図をもらって各セルの情報からStateのオブジェクトを作ってstatesというベクトルに突っ込む */
ValueIterator::ValueIterator(nav_msgs::OccupancyGrid &map)
{
	_cell_x_num = map.info.width;
	_cell_y_num = map.info.height;
	_cell_t_num = 60; //6[deg]刻みでとりあえず固定

	_cell_x_width = map.info.resolution;
	_cell_y_width = map.info.resolution;
	_cell_t_width = 360/_cell_t_num;

	_center_state_ix = _cell_x_num/2;
	_center_state_iy = _cell_y_num/2;

	_final_state_x = 0.0;
	_final_state_y = 0.0;
	_final_state_width = 0.5;

	for(int y=0; y<_cell_y_num; y++)
		for(int x=0; x<_cell_x_num; x++)
			for(int t=0; t<_cell_t_num; t++)
				_states.push_back(State(x, y, t, map.data[y*_cell_x_num + x]));

	setStateValues();

	outputValuePgmMap();

	setAction();
	setStateTransition();


	_delta = fabs(_value_min);

	/*
	vector<thread> ths;
	for(int t=0; t<12; t++)
		ths.push_back(thread(&ValueIterator::valueIterationWorker, this));

	for(auto &th : ths)
		th.join();

	outputValuePgmMap();
	exit(0);
	*/
}

/* デフォルトのアクションの設定 */
void ValueIterator::setAction(void)
{
	_actions.push_back(Action("forward", 0.1, 0.0));
	_actions.push_back(Action("right", 0.0, -10.0));
	_actions.push_back(Action("left", 0.0, 10.0));
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
	const int x_step = 100;
	const int y_step = 100;
	const int t_step = 100;
	const double prob_quota = 1.0/(x_step*y_step*t_step);

	double theta_origin = it*_cell_t_width;

	for(int y=0; y<y_step; y++){
		for(int x=0; x<x_step; x++){
			for(int t=0; t<t_step; t++){
				//遷移前の姿勢
				double ox = x*_cell_x_width/x_step;
				double oy = y*_cell_y_width/y_step;
				double ot = t*_cell_t_width/t_step + theta_origin;

				//遷移後の姿勢
				double dx, dy, dt;
				accurateStateTransition(a, ox, oy, ot, dx, dy, dt);
				int dix, diy, dit;
				toCellPos(dx, dy, dt, dix, diy, dit); 

				bool exist = false;
				for(auto &s : a._state_transitions[it]){
					if(s._dix == dix and s._diy == diy and s._dit == dit){
						s._prob += prob_quota;
						exist = true;
						break;
					}
				}
				if(not exist)
					a._state_transitions[it].push_back(StateTransition(dix, diy, dit, prob_quota));
			}
		}
	}
}

double ValueIterator::valueIteration(State &s)
{
	if((not s._free) or s._final_state)
		return 0.0;

	double max_value = ValueIterator::_value_min*10;
	for(auto a : _actions){
		double q = actionValue(s, a);
		if(q > max_value)
			max_value = q;
	}

	if(max_value < ValueIterator::_value_min)
		max_value = ValueIterator::_value_min;

	double delta = fabs(max_value - s._value);
	s._value = max_value;

	return delta;
}

void ValueIterator::valueIterationWorker(int times)
{
	cout << "address:" << &_states[0] << endl;
	for(int j=0; j<times; j++){
		double max_delta = 0.0;
	
		int start = rand()%_states.size();
		for(int i=start; i<_states.size(); i++){
			double delta = valueIteration(_states[i]);
			max_delta = (max_delta > delta) ? max_delta : delta;
		}
		
		outputValuePgmMap();
	
		for(int i=0; i<start; i++){
			double delta = valueIteration(_states[i]);
			max_delta = (max_delta > delta) ? max_delta : delta;
		}
	
		_delta = max_delta;
		cout << "delta: " << _delta << endl;
		if(_delta < 0.1)
			break;
	}
}

int ValueIterator::toIndex(int ix, int iy, int it)
{
	return it + ix*_cell_t_num + iy*(_cell_t_num*_cell_x_num);
}

double ValueIterator::actionValue(State &s, Action &a)
{
	double value = 0.0;
	for(auto &tran : a._state_transitions[s._it]){
		int ix = s._ix + tran._dix;
		if(ix < 0 or ix >= _cell_x_num)
			return -10000000.0;

		int iy = s._iy + tran._diy;
		if(iy < 0 or iy >= _cell_y_num)
			return -10000000.0;

		int it = (s._it + tran._dit + _cell_t_num)%_cell_t_num;

		auto &after_s = _states[toIndex(ix, iy, it)];
		if(not after_s._free)
			return -10000000.0;

		value += after_s._value * tran._prob;
	}

	return value - 1.0;
}

/* statesのセルの情報をPBMとして出力（デバッグ用） */
void ValueIterator::outputPbmMap(void){
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

void ValueIterator::setStateValues(void)
{
	for(auto &s : _states){
		double x0 = (s._ix - _center_state_ix)*_cell_x_width;
		double y0 = (s._iy - _center_state_iy)*_cell_y_width;
		double x1 = x0 + _cell_x_width;
		double y1 = y0 + _cell_y_width;

		s._final_state = fabs(x0 - _final_state_x) < _final_state_width
			       && fabs(y0 - _final_state_y) < _final_state_width
			       && fabs(x1 - _final_state_y) < _final_state_width 
			       && fabs(y1 - _final_state_y) < _final_state_width
			       && s._free;
	}

	for(auto &s : _states){
		if(s._final_state)
			s._value = 0.0;
		else
			s._value = _value_min;
	}
}

void ValueIterator::outputValuePgmMap(void)
{
	for(int t=0; t<_cell_t_num; t++){
		ofstream ofs("/tmp/value_t=" + to_string(t) + ".pgm");

		ofs << "P2" << endl;
		ofs << _cell_x_num << " " << _cell_y_num << " 255" << endl;
		int i = t;
		while(i<_states.size()){
			if(_states[i]._free and _states[i]._value >= -255.0)
				ofs << 255 - (int)fabs(_states[i]._value) << " ";
			else
				ofs << 0 << " ";

			i += _cell_t_num;
		}
	
		ofs << flush;
	}
}
