#include "ValueIterator.h"
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

	_delta_fw_stdev = fabs(fw)*0.1;
	_delta_rot_stdev = fabs(rot)*0.1;
}

/* ROSの地図をもらって各セルの情報からStateのオブジェクトを作ってstatesというベクトルに突っ込む */
ValueIterator::ValueIterator(nav_msgs::OccupancyGrid &map)
{
	_cell_x_num = map.info.width;
	_cell_y_num = map.info.height;
	_cell_t_num = 72;

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
}

/* デフォルトのアクションの設定 */
void ValueIterator::setAction(void)
{
	_actions.push_back(Action("forward", 0.3, 0.0));
	_actions.push_back(Action("right", 0.0, -10.0));
	_actions.push_back(Action("left", 0.0, 10.0));
}

void ValueIterator::setStateTransition(void)
{
	for(auto &a : _actions){
		setStateTransition(a);
	}
}

void ValueIterator::setStateTransition(Action &a)
{
	const int x_step = 1000;
	const int y_step = 1000;
	const int t_step = 1000;

	
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
