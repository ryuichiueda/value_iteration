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
	State(int x, int y, int theta, int map_value);

	double _value;
	int _ix, _iy, _it;
	bool _free;
	bool _final_state;
};

class ValueIterator{
private: 
	vector<State> _states;
	double _cell_x_width, _cell_y_width, _cell_t_width;
	int _cell_x_num, _cell_y_num, _cell_t_num;

	int _center_state_ix, _center_state_iy;

	double _final_state_x, _final_state_y, _final_state_width;
public: 
	ValueIterator(nav_msgs::OccupancyGrid &map);
	void outputPbmMap(void);
	void setFinalState(void);
	void outputValuePgmMap(void);
};

#endif
