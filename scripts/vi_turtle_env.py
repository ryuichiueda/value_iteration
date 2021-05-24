#! /usr/bin/env python3

import rospy
from value_iteration.msg import ViAction, ViGoal
from geometry_msgs.msg import PoseStamped
import actionlib

def vi_state_cb(feedback):
    rospy.loginfo("SWEEPS: " + ", ".join(str(e) for e in feedback.current_sweep_times.data))
    rospy.loginfo("DELTAS: " + ", ".join(str(e) for e in feedback.deltas.data))

def vi_client(data):
    client = actionlib.SimpleActionClient('/vi_controller', ViAction)
    client.wait_for_server()

    goal = ViGoal()
    goal.goal = data
    client.send_goal(goal, feedback_cb=vi_state_cb)
    client.wait_for_result()
    return client.get_result()

def receive_goal(data):
    rospy.loginfo(data)
    result = vi_client(data)
    rospy.loginfo(result)

if __name__ == '__main__':
    rospy.init_node('vi_controller_turtle_env')

    sub_goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, receive_goal)

    rospy.spin()
