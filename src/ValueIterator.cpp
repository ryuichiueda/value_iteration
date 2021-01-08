#include "ValueIterator.h"
using namespace std;

State::State(int x, int y, int map_value)
{
	ix = x;
	iy = y;
	free = (map_value == 0);
}

/* ROSの地図をもらって各セルの情報からStateのオブジェクトを作ってstatesというベクトルに突っ込む */
ValueIterator::ValueIterator(nav_msgs::OccupancyGrid &map)
{
	_width = map.info.width;
	_height = map.info.height;

	for(int y=0;y<_height;y++){
		for(int x=0;x<_width;x++){
			_states.push_back(State(x, y, map.data[y*_width + x]));
		}
	}
}

/* statesのセルの情報をPBMとして出力（デバッグ用） */
void ValueIterator::outputPbmMap(void){
	ofstream ofs("/tmp/a.pbm");

	ofs << "P1" << endl;
	ofs << _width << " " << _height << endl;
	for(auto s : _states)
		ofs << s.free << " ";

	ofs << flush;
}
