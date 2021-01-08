#ifndef VALUE_ITERATOR_
#define VALUE_ITERATOR_

#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include <vector>
#include <fstream>
using namespace std;

class State{
public: 
	State(int x, int y, int map_value);

	unsigned int ix, iy;
	bool free;
};

class ValueIterator{
private: 
	vector<State> _states;
	unsigned int _width, _height;
	double _cell_size;
public: 
	ValueIterator(nav_msgs::OccupancyGrid &map);
	void outputPbmMap(void);
};

#endif
