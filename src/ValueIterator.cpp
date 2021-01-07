#include "ValueIterator.h"
using namespace std;

State::State(int x, int y, int map_value)
{
	ix = x;
	iy = y;
	free = (map_value == 0);
}

ValueIterator::ValueIterator(nav_msgs::OccupancyGrid &map)
{
	width = map.info.width;
	height = map.info.height;

	for(int y=0;y<map.info.height;y++){
		for(int x=0;x<map.info.width;x++){
			states.push_back(State(x, y, map.data[y*map.info.width + x]));
		}
	}
}

void ValueIterator::outputPbmMap(void){
	ofstream ofs("/tmp/a.pbm");

	ofs << "P1" << endl;
	ofs << width << " " << height << endl;
	for(auto s : states)
		ofs << s.free << " ";

	ofs << flush;
}
