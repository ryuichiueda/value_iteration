#include "value_iteration/vi_node_no_local.h"

namespace value_iteration{

ViNode::ViNode() : private_nh_("~"), yaw_(0.0), x_(0.0), y_(0.0), online_("false")
{
	setActions();

	int thread_num;
	private_nh_.param("thread_num", thread_num, 1);
	vi_.reset(new ValueIterator(*actions_, thread_num));

	private_nh_.param("cost_drawing_threshold", cost_drawing_threshold_, 60);

	setCommunication();

	nav_msgs::GetMap::Response res;
	setMap(res);
}

ViNode::~ViNode() 
{
	delete actions_;
}

void ViNode::setMap(nav_msgs::GetMap::Response &res)
{
	int theta_cell_num;
	double safety_radius;
	double safety_radius_penalty;
	double goal_margin_radius;
	int goal_margin_theta;

	private_nh_.param("theta_cell_num", theta_cell_num, 60);
	private_nh_.param("safety_radius", safety_radius, 0.2);
	private_nh_.param("safety_radius_penalty", safety_radius_penalty, 30.0);
	private_nh_.param("goal_margin_radius", goal_margin_radius, 0.2);
	private_nh_.param("goal_margin_theta", goal_margin_theta, 10);

	std::string map_type;
	private_nh_.param("map_type", map_type, std::string("occupancy"));

	if(map_type == "occupancy"){
		while(!ros::service::waitForService("/static_map", ros::Duration(3.0))){
			ROS_INFO("Waiting for static_map");
		}

		ros::ServiceClient client = nh_.serviceClient<nav_msgs::GetMap>("/static_map");
		nav_msgs::GetMap::Request req;
		if(not client.call(req, res)){
			ROS_ERROR("static_map not working");
			exit(1);
		}
	
		vi_->setMapWithOccupancyGrid(res.map, theta_cell_num, safety_radius, safety_radius_penalty,
			goal_margin_radius, goal_margin_theta);
	}else if(map_type == "cost"){
		while(!ros::service::waitForService("/cost_map", ros::Duration(3.0))){
			ROS_INFO("Waiting for cost_map");
		}

		ros::ServiceClient client = nh_.serviceClient<nav_msgs::GetMap>("/cost_map");
		nav_msgs::GetMap::Request req;
		if(not client.call(req, res)){
			ROS_ERROR("cost_map not working");
			exit(1);
		}
		for(int i=0;i<100;i++)
			ROS_INFO("%u", (unsigned int)(res.map.data[i] & 0xFF));
	
		vi_->setMapWithCostGrid(res.map, theta_cell_num, safety_radius, safety_radius_penalty,
			goal_margin_radius, goal_margin_theta);
	}else{
		ROS_ERROR("NO SUPPORT MAP TYPE");
		exit(1);
	}
}

void ViNode::setCommunication(void)
{
	private_nh_.param("online", online_, false);
	if(online_){
		ROS_INFO("SET ONLINE");
		pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 2, true);
		//sub_pose_ = nh_.subscribe("mcl_pose", 2, &ViNode::poseReceived, this);
		sub_laser_scan_ = nh_.subscribe("scan", 2, &ViNode::scanReceived, this);
	}

	pub_value_function_ = nh_.advertise<nav_msgs::OccupancyGrid>("value_function", 2, true);
	pub_local_value_function_ = nh_.advertise<nav_msgs::OccupancyGrid>("local_value_function", 2, true);

	as_.reset(new actionlib::SimpleActionServer<value_iteration::ViAction>( nh_, "vi_controller",
				boost::bind(&ViNode::executeVi, this, _1), false));
	as_->start();
	srv_policy_ = nh_.advertiseService("/policy", &ViNode::servePolicy, this);
	srv_value_ = nh_.advertiseService("/value", &ViNode::serveValue, this);
}

void ViNode::setActions(void)
{
	XmlRpc::XmlRpcValue params;
	nh_.getParam("/vi_node", params);
	ROS_ASSERT(params.getType() == XmlRpc::XmlRpcValue::TypeStruct);
	auto &action_list = params["action_list"];
	ROS_ASSERT(action_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	actions_ = new vector<Action>();

	for(int i=0; i<action_list.size(); i++){
		auto &a = action_list[i];
		actions_->push_back(Action(a["name"], a["onestep_forward_m"], a["onestep_rotation_deg"], i));

		auto &b = actions_->back();
		ROS_INFO("set an action: %s, %f, %f", b._name.c_str(), b._delta_fw, b._delta_rot);
	}
}

void ViNode::scanReceived(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	//vi_->setLocalCost(msg, x_, y_, yaw_);
}

bool ViNode::servePolicy(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response)
{
	vi_->policyWriter(response);
	return true;
}

bool ViNode::serveValue(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response)
{
	vi_->valueFunctionWriter(response);
	return true;
}

void ViNode::executeVi(const value_iteration::ViGoalConstPtr &goal)
{
	static bool executing = true;
	ROS_INFO("VALUE ITERATION START");
	auto &ori = goal->goal.pose.orientation;	
	tf::Quaternion q(ori.x, ori.y, ori.z, ori.w);
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	int t = (int)(yaw*180/M_PI);
	vi_->setGoal(goal->goal.pose.position.x, goal->goal.pose.position.y, t);

	vector<thread> ths;
	for(int t=0; t<vi_->thread_num_; t++)
		ths.push_back(thread(&ValueIterator::valueIterationWorker, vi_.get(), INT_MAX, t));

	value_iteration::ViFeedback vi_feedback;

	ros::Rate loop_rate(10);
	while(not vi_->finished(vi_feedback.current_sweep_times, vi_feedback.deltas)){
		as_->publishFeedback(vi_feedback);

		if(as_->isPreemptRequested())
			vi_->setCancel();

		loop_rate.sleep();
	}
	as_->publishFeedback(vi_feedback);

	for(auto &th : ths)
		th.join();

	ROS_INFO("VALUE ITERATION END");
	while(online_ and not vi_->endOfTrial() )
		if(as_->isPreemptRequested()){
			vi_->setCancel();

		loop_rate.sleep();
	}

	ROS_INFO("END OF TRIAL");
	value_iteration::ViResult vi_result;
	vi_result.finished = vi_->arrived() or (not online_);
	as_->setSucceeded(vi_result);
}


void ViNode::pubValueFunction(void)
{
	nav_msgs::OccupancyGrid map;//, local_map;

	vi_->makeValueFunctionMap(map, cost_drawing_threshold_, x_, y_, yaw_);
	pub_value_function_.publish(map);

	//vi_->makeLocalValueFunctionMap(local_map, cost_drawing_threshold_, x_, y_, yaw_);
	//pub_local_value_function_.publish(local_map);
}

void ViNode::decision(void)
{
	if(not online_)
		return; 

	try{
		tf::StampedTransform trans;
		tf_listener_.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(0.1));
		tf_listener_.lookupTransform("map", "base_link", ros::Time(0), trans);
		x_ = trans.getOrigin().x();
		y_ = trans.getOrigin().y();
		yaw_ = tf::getYaw(trans.getRotation());
	}catch(tf::TransformException &e){
		ROS_WARN("%s", e.what());
	}

	//vi_->setLocalWindow(x_, y_);

	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = 0.0;
	cmd_vel.angular.z = 0.0;

	Action *a = vi_->posToAction(x_, y_, yaw_);
	if(a != NULL){
		cmd_vel.linear.x = a->_delta_fw;
		cmd_vel.angular.z = a->_delta_rot/180*M_PI;
	}
	pub_cmd_vel_.publish(cmd_vel);
}

}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"vi_node");
	value_iteration::ViNode vi_node;

	int step = 0;
	ros::Rate loop_rate(10);
	while(ros::ok()){
		vi_node.decision();
		
		if(step % 30 == 0)
			vi_node.pubValueFunction();

		ros::spinOnce();
		loop_rate.sleep();
		step++;
	}

	return 0;
}
