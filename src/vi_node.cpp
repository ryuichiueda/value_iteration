#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"

/*
#include "Event.h"
#include "pfoe/FlushData.h"
#include "pfoe/EventRegist.h"
#include "pfoe/ActionRegist.h"
#include "ParticleFilter.h"
*/
#include <iostream>
#include <fstream>
using namespace ros;

int main(int argc, char **argv)
{
	init(argc,argv,"vi_node");
	NodeHandle n;

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

	for(int y=0;y<res.map.info.height;y++)
		for(int x=0;x<res.map.info.width;x++)
			ROS_INFO("CELL %d %d %d", x, y, res.map.data[y*res.map.info.width + x]);

	spin();

	return 0;
}

/*
Episode episode(1000,0.99);
ParticleFilter pf(1000);

bool action_regist(pfoe::ActionRegist::Request &req, pfoe::ActionRegist::Response &res)
{
	res.ok = pf.registAction(req.action);
	return true;
}

bool event_regist(pfoe::EventRegist::Request &req, pfoe::EventRegist::Response &res)
{
	episode.push_back(Event(req.action,req.sensor,req.reward));
	res.decision = "fw";

	if(episode.size() < 2)
		return true;

	pf.update(&episode);
	res.decision = pf.decision(&episode);

	system("clear");
	return true;
}

bool flush_data(pfoe::FlushData::Request &req, pfoe::FlushData::Response &res)
{
	ofstream ofs(req.file);
	if(req.type == "episode"){
		ROS_INFO("Episode is flushed to %s.", req.file.c_str());
		episode.flushData(&ofs);

		res.ok = true;
	}else if(req.type == "particles"){
		ROS_INFO("Particles' data is flushed to %s.", req.file.c_str());
		pf.print(&ofs);
	}else
		res.ok = false;

	ofs.close();

	return true;
}

int main(int argc, char **argv)
{
	init(argc,argv,"pfoe_node");
	NodeHandle n;

	ros::ServiceServer s1 = n.advertiseService("event_regist", event_regist);
	ros::ServiceServer sa = n.advertiseService("action_regist", action_regist);
	ros::ServiceServer s2 = n.advertiseService("flush_data", flush_data);
	ROS_INFO("Ready to regist events.");

	spin();

	return 0;
}
*/
