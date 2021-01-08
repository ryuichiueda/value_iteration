#include "ValueIterator.h"
using namespace std;

State::State(int x, int y, int theta, int map_value)
{
	_ix = x;
	_iy = y;
	_free = (map_value == 0);
}

/* ROSの地図をもらって各セルの情報からStateのオブジェクトを作ってstatesというベクトルに突っ込む */
ValueIterator::ValueIterator(nav_msgs::OccupancyGrid &map)
{
	_cell_x_num = map.info.width;
	_cell_y_num = map.info.height;
	_cell_t_num = 72;

	_cell_x_width = map.info.resolution;
	_cell_y_width = map.info.resolution;
	_cell_t_width = 3.141592/72;

	for(int y=0;y<_cell_y_num;y++){
		for(int x=0;x<_cell_x_num;x++){
			_states.push_back(State(x, y, 0, map.data[y*_cell_x_num + x]));
		}
	}
}

/* statesのセルの情報をPBMとして出力（デバッグ用） */
void ValueIterator::outputPbmMap(void){
	ofstream ofs("/tmp/a.pbm");

	ofs << "P1" << endl;
	ofs << _cell_x_num << " " << _cell_y_num << endl;
	for(auto s : _states)
		ofs << s._free << " ";

	ofs << flush;
}
