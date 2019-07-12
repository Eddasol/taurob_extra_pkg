#!/usr/bin/env python

'''
Create node "vel_control"
Subscribes to the topic "taurob_tracker/desired_nav_cmd_vel" alters the message received to a more realistic value and publishes it to "taurob_tracker/cmd_vel_raw"
'''
import rospy
import threading
import thread
from geometry_msgs.msg import Twist
from numpy import sign


def alter_vel(msg):
    min_only_angular = 0.7
    approx_zero_vel = 0.1
    z = msg.angular.z
    x = msg.linear.x


    if z == -0.4 and x == 0:
        msg.angular.z = -0.7
    elif z == 0.4 and x == 0:
        msg.angular.z = 0.7

    return msg

    # Won't run !!!

    # If the robot is only rotating and the angular velocity is too low the system can be inf slow
    if msg.linear.x == 0 and msg.angular.z == 0:
        # Case 1: Do nothing
        return msg
    elif abs(msg.linear.x) < approx_zero_vel and abs(msg.angular.z) < min_only_angular:
        # Case 2: Change heading, but to low value
        msg.angular.z = sign(msg.angular.z)*min_only_angular
        print("Case 2: Too low angular value")
    return msg


def pub_new_vel(msg):
    pub = rospy.Publisher('taurob_tracker/cmd_vel_raw', Twist, queue_size=1)
    pub.publish(alter_vel(msg))


def callback(msg):
    prev_msg = msg
    pub_new_vel(msg)
    return "done"


def read_vel():
    rospy.init_node('vel_control')
    rospy.Subscriber("taurob_tracker/desired_nav_cmd_vel", Twist, callback)
    rospy.spin()


if __name__=="__main__":
    read_vel()
