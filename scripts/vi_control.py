#! /usr/bin/env python3

import rospy
from value_iteration.msg import ViAction, ViGoal
import actionlib

def vi_state_cb(feedback):
    print(feedback)

def vi_client():
    client = actionlib.SimpleActionClient('/vi_controller', ViAction)
    client.wait_for_server()

    goal = ViGoal()
    goal.sweepnum = 0
    goal.threadnum = 4
    client.send_goal(goal, feedback_cb=vi_state_cb)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('vi_controller')
    result = vi_client()
    print(result)

    rospy.spin()
