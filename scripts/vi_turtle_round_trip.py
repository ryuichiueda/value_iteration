#! /usr/bin/env python3

import rospy, math, time, tf
from value_iteration.msg import ViAction, ViGoal
from geometry_msgs.msg import PoseStamped
import actionlib

pos = [(6.0, -2.0, math.pi/2), 
        (3.0, 3.0, math.pi/3),
        (4.0, -1.0, math.pi*1.5), 
        (1.0, 2.0, 0.0),
        (-4.0, -1.0, 0.0),
        (-2.0, 4.5, math.pi),
        (-7.0, -3.0, math.pi/4*3),
        (-7.0, 2.0, math.pi/4*7) ]

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

#def receive_goal(data):
#    result = vi_client(data)
#    rospy.loginfo(result)

if __name__ == '__main__':
    time.sleep(15)
    rospy.init_node('vi_controller_turtle_env')

    for p in pos:
        goal = PoseStamped()
        goal.pose.position.x = p[0]
        goal.pose.position.y = p[1]
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, p[2])
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        vi_client(goal)

    #rospy.spin()
