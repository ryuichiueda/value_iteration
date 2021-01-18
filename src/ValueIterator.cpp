#include "ValueIterator.h"
#include <thread>
using namespace std;

State::State(int x, int y, int theta, int map_value)
{
	_ix = x;
	_iy = y;
	_value = -100.0;
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
	_final_state_width = 1.0;

	for(int y=0; y<_cell_y_num; y++)
		for(int x=0; x<_cell_x_num; x++)
			for(int t=0; t<_cell_t_num; t++)
				_states.push_back(State(x, y, t, map.data[y*_cell_x_num + x]));

	setFinalState();
	setAction();
	setStateTransition();

	for(auto &a : _actions){
		for(int t=0; t<_cell_t_num; t++){
			auto ss = a._state_transitions[t];
			for(auto s : ss)
				cout << a._name << "\ttheta:" << t << "\t" << s.to_string() << endl;
		}
	}
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
	vector<thread> ths;

	vector<StateTransition> theta_state_transitions;
	for(auto &a : _actions){
		for(int t=0; t<_cell_t_num; t++){
			//cout << a._name << " " << t << endl;
			a._state_transitions.push_back(theta_state_transitions);
			//setStateTransition(a, t);
		}
	}


	for(int t=0; t<_cell_t_num; t++){
		ths.push_back(thread(&ValueIterator::setStateTransitionWorker, this, t));
	}

	for(auto &th : ths)
		th.join();
}/

void ValueIterator::setStateTransitionWorker(int it)
{
	const int x_step = 100;
	const int y_step = 100;
	const int t_step = 100;
	const double prob_quota = 1.0/(x_step*y_step*t_step);

	double theta_origin = it*_cell_t_width;

	for(auto &a : _actions){
		setStateTransition(a, it);
	}
}

void ValueIterator::setStateTransition(Action &a, int it)
{
	const int x_step = 100;
	const int y_step = 100;
	const int t_step = 100;
	const double prob_quota = 1.0/(x_step*y_step*t_step);

	double theta_origin = it*_cell_t_width;


	//XY平面での遷移（thetaごと）
	for(int y=0; y<y_step; y++){
		for(int x=0; x<x_step; x++){
			for(int t=0; t<t_step; t++){
				//遷移前の姿勢
				double ox = x*_cell_x_width/x_step;
				double oy = y*_cell_y_width/y_step;
				double ot = t*_cell_t_width/t_step + theta_origin;
				double ot_rad = ot * 3.141592/180;

				//遷移後の姿勢
				double dx = ox + a._delta_fw*cos(ot_rad);
				double dy = oy + a._delta_fw*sin(ot_rad);
				double dt = ot + a._delta_rot;
				while(dt < 0.0)
					dt += 360.0;

				//遷移後の離散状態
				//int dix = ((int)floor(fabs(dx) / _cell_x_width))*(dx < 0.0 ? -1 : 1);
				//int diy = ((int)floor(fabs(dy) / _cell_y_width))*(dy < 0.0 ? -1 : 1);
				int dix = (int)floor(fabs(dx) / _cell_x_width);
				if(dx < 0.0)
					dix = -dix-1;
				int diy = (int)floor(fabs(dy) / _cell_y_width);
				if(dy < 0.0)
					diy = -diy-1;

				int dit = (int)floor(dt / _cell_t_width);

				//cout << dix << " " << dx << " " << _cell_x_width << endl;

				bool exist = false;
				for(auto &s : a._state_transitions[it]){
					if(s._dix == dix and s._diy == diy and s._dit == dit){
						s._prob += prob_quota;
						exist = true;
					}
				}
				if(not exist)
					a._state_transitions[it].push_back(StateTransition(dix, diy, dit, prob_quota));
			}
		}
	}
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

void ValueIterator::setFinalState(void)
{
	for(auto &s : _states){
		double x0 = (s._ix - _center_state_ix)*_cell_x_width;
		double y0 = (s._iy - _center_state_iy)*_cell_y_width;
		double x1 = x0 + _cell_x_width;
		double y1 = y0 + _cell_y_width;

		s._final_state = fabs(x0 - _final_state_x) < _final_state_width
			       && fabs(y0 - _final_state_y) < _final_state_width
			       && fabs(x1 - _final_state_y) < _final_state_width 
			       && fabs(y1 - _final_state_y) < _final_state_width;

		if(s._final_state)
			s._value = 0.0;
	}
}

void ValueIterator::outputValuePgmMap(void)
{
	double min_value = 0.0;
	for(auto &s : _states){
		if(min_value > s._value)
			min_value = s._value;
	}

	for(int t=0; t<_cell_t_num; t++){
		ofstream ofs("/tmp/value_t=" + to_string(t) + ".pgm");
	
		ofs << "P2" << endl;
		ofs << _cell_x_num << " " << _cell_y_num << " 255" << endl;
		int i = t;
		while(i<_states.size()){
			ofs << 255 - int(_states[i]._value/min_value*255) << " ";
			i += _cell_t_num;
		}
	
		ofs << flush;
	}
}
