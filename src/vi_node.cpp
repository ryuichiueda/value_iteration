#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include <iostream>
#include <vector>
using namespace std;

class State{
public: 
	State(int x, int y, int map_value){
		ix = x;
		iy = y;
		free = (map_value == 0);
	}

	unsigned int ix, iy;
	bool free;
};

class ValueIterator{
private: 
	vector<State> states;
	unsigned int width, height;
public: 
	ValueIterator(nav_msgs::OccupancyGrid &map){
		width = map.info.width;
		height = map.info.height;

		for(int y=0;y<map.info.height;y++){
			for(int x=0;x<map.info.width;x++){
				states.push_back(State(x, y, map.data[y*map.info.width + x]));
			}
		}
	}

	void outputPbmMap(void){
		cout << "P1" << endl;
		cout << width << " " << height << endl;
		for(auto s : states)
			cout << s.free << " ";

		cout << flush;
	};
};

int main(int argc, char **argv)
{
	ros::init(argc,argv,"vi_node");
	ros::NodeHandle n;

	while(!ros::service::waitForService("/static_map", ros::Duration(3.0))){
		ROS_INFO("Waiting for static_map");
	}

	ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("/static_map");

	nav_msgs::GetMap::Request req;
	nav_msgs::GetMap::Response res;
	if(not client.call(req, res)){
		ROS_ERROR("static_map not working");
		return 1;
	}

	ValueIterator value_iterator(res.map);

	value_iterator.outputPbmMap();

	//ros::spin();

	return 0;
}
