#! /usr/bin/env python3

import rospy
from value_iteration.msg import ViAction, ViGoal
import actionlib

def vi_state_cb(feedback):
    print(feedback)

def vi_client():
    print("enter")
    client = actionlib.SimpleActionClient('/vi_controller', ViAction)
    print("waiting")
    client.wait_for_server()
    print("ok")

    goal = ViGoal()
    goal.sweepnum = 0
    goal.threadnum = 2
    client.send_goal(goal, feedback_cb=vi_state_cb)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('vi_controller')
    result = vi_client()
    print(result)
