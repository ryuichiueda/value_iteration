#include <ros/ros.h>
#include "ValueIterator.h"

#include <gtest/gtest.h>

// Declare a test
TEST(ValueIteratorTest, testCase1)
{
	EXPECT_EQ(1,1);
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc,argv,"tester");
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

	return RUN_ALL_TESTS();
}
