#include "ValueIterator.h"
using namespace std;

State::State(int x, int y, int theta, int map_value)
{
	_ix = x;
	_iy = y;
	_value = 100.0;
	_free = (map_value == 0);
	_final_state = false;
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

	_final_state_x = 0.0;
	_final_state_y = 0.0;
	_final_state_width = 1.0;

	for(int y=0; y<_cell_y_num; y++)
		for(int x=0; x<_cell_x_num; x++)
			for(int t=0; t<_cell_t_num; t++)
				_states.push_back(State(x, y, t, map.data[y*_cell_x_num + x]));
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
