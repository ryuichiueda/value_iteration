#include "value_iteration/vi_node.h"

namespace value_iteration{

ViNode::ViNode() : private_nh_("~") 
{
	setActions();

	int thread_num;
	private_nh_.param("thread_num", thread_num, 1);
	vi_.reset(new ValueIterator(*actions_, thread_num));

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
}

void ViNode::setCommunication(void)
{
	bool online;
	private_nh_.param("online", online, false);
	if(online){
		ROS_INFO("SET ONLINE");
		pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 2, true);
		sub_pose_ = nh_.subscribe("mcl_pose", 2, &ViNode::poseReceived, this);
	}

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

void ViNode::poseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
	auto &ori = msg->pose.pose.orientation;	
	tf::Quaternion q(ori.x, ori.y, ori.z, ori.w);
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	Action *a = vi_->posToAction(msg->pose.pose.position.x,
				msg->pose.pose.position.y, yaw);

	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = 0.0;
	cmd_vel.angular.z = 0.0;
	if(a != NULL){
		cmd_vel.linear.x = a->_delta_fw;
		cmd_vel.angular.z= a->_delta_rot/180*M_PI;
	}
	pub_cmd_vel_.publish(cmd_vel);
}

bool ViNode::servePolicy(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response)
{
	vi_->actionImageWriter(response);
	return true;
}

bool ViNode::serveValue(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response)
{
	vi_->outputValuePgmMap(response);
	return true;
}

void ViNode::executeVi(const value_iteration::ViGoalConstPtr &goal)
{
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
	while(not vi_->finished(vi_feedback.current_sweep_times, vi_feedback.deltas)){
		as_->publishFeedback(vi_feedback);
		sleep(1);
	}
	as_->publishFeedback(vi_feedback);

	for(auto &th : ths)
		th.join();

	value_iteration::ViResult vi_result;
	vi_result.finished = true;
	as_->setSucceeded(vi_result);
}

}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"vi_node");
	value_iteration::ViNode vi_node;

	ros::Rate loop_rate(1);
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
