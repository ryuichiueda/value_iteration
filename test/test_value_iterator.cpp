#include "ValueIterator.h"
#include "include/Valueiterator_GoogleTest.h"

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

/*
/	outputPbmMap()のテスト
*/
TEST_F(gtest_diff, gtest_file_diff2)
{
	gtest_diff *test;

	string file_path1, file_path2;
	file_path1 = _map_path;
	file_path2 = "/tmp/a.pbm";

	gtest_file_diff2(file_path1, file_path2);
    
	ASSERT_TRUE(_return);
}
