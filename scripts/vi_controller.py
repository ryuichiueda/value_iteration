#! /usr/bin/env python3

import rospy, copy
from value_iteration.msg import ViAction, ViGoal
from geometry_msgs.msg import PoseStamped
import actionlib

goal_data = None
client = None

def vi_state_cb(feedback):
    pass
    #rospy.loginfo("SWEEPS: " + ", ".join(str(e) for e in feedback.current_sweep_times.data))
    #rospy.loginfo("DELTAS: " + ", ".join(str(e) for e in feedback.deltas.data))

def vi_client(data):
    global client 
    global goal_data

    client = actionlib.SimpleActionClient('/vi_controller', ViAction)
    client.wait_for_server()

    goal = ViGoal()
    goal.goal = copy.copy(data)
    client.send_goal(goal, feedback_cb=vi_state_cb)
    client.wait_for_result()
    return client.get_result()

def receive_goal(data):
    global goal_data
    global client

    if goal_data != None:
        client.cancel_goal()

    goal_data = data

if __name__ == '__main__':
    rospy.init_node('vi_controller_turtle_env')

    sub_goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, receive_goal)

    rate = rospy.Rate(20) 
    while not rospy.is_shutdown():
        if goal_data == None:
            rate.sleep()
            continue

        result = vi_client(goal_data)
        if result.finished:
            goal_data = None
        rospy.loginfo(result)

        rate.sleep()

    rospy.spin()
