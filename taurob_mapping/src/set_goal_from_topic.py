#!/usr/bin/env python
'''
Example of controlscript that desides which topic that controls the velocity
'''

# This file will only shutdown if it has a goal position
import roslib
import rospy
import actionlib
import math
import threading
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from threading import Semaphore
from rospy import loginfo as rosinfo
from time import time, sleep



client = None
available_goal = Semaphore(value=0)


DEFAULT = "cmd_vel_default"
OVERRIDE = "cmd_vel_override"
writer = DEFAULT

normal_drift_time_out = None
recovery_time_out = None

min_speed = 0.4
recovery_max_time = 6
recovery_max_forward_time = 2
normal_max_no_progress_time = 6

def set_recovery_time_out(forward = False):
	recover_lock.acquire()
	#print("---Reset recovery timer")
	global recovery_time_out
	recovery_time_out = time() + recovery_max_time
	recover_lock.release()


def reset_normal_drift_time_out():
	normal_lock.acquire()

	global normal_drift_time_out
	normal_drift_time_out = time() + normal_max_no_progress_time
	normal_lock.release()

def update_writer(_writer):
	global writer
	writer = _writer

def publish_writer():
	info = String()
	info.data = writer
	pub_writer.publish(info)
	
def deside_writer(data):
	x = data.linear.x
	
	if normal_drift_time_out is None:
		reset_normal_drift_time_out()
	if recovery_time_out is None:
		set_recovery_time_out()

	if x > min_speed and writer == DEFAULT:
		# normal drift
		reset_normal_drift_time_out()
	elif normal_drift_time_out < time() and writer == DEFAULT:
		# recovery started
		set_recovery_time_out()
		update_writer(OVERRIDE)
	elif recovery_time_out < time() and writer == OVERRIDE:
		# recovery ended
		reset_normal_drift_time_out()
		update_writer(DEFAULT)
	#elif x > min_speed and writer == OVERRIDE:
		# starts driving forward in recovery
	#	set_recovery_time_out(forward = True)
	
	publish_writer()

	
	


def callback(goal):
    # sets a new timestamp, but otherwise sends the same message
    goal.target_pose.header.stamp = rospy.Time.now()
    rosinfo("Sending new goal (%s, %s)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
    client.send_goal(goal)

    available_goal.release()



if __name__ == '__main__':
    # Initializing
    rospy.init_node('set_nav_goal')
    rospy.Subscriber("goals", MoveBaseGoal, callback)
    rospy.Subscriber("taurob_tracker/cmd_vel_raw", Twist, deside_writer)
    pub_writer = rospy.Publisher("navigation_status", String, queue_size = 1)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    recover_lock = threading.Lock()
    normal_lock = threading.Lock()

    # Wait for connection
    while (not client.wait_for_server(rospy.Duration.from_sec(5.0))) and not rospy.is_shutdown():
        rosinfo("Waiting for the move_base action server to come up")
    rosinfo("Connected")

    # Give feedback to user
    while not rospy.is_shutdown():
        available_goal.acquire()
	if rospy.is_shutdown():
		break

	# Resets when getting a new goal
	reset_normal_drift_time_out()
	update_writer(DEFAULT)

        client.wait_for_result()

        if (client.get_state() == actionlib.GoalStatus.SUCCEEDED):
            rosinfo("Hooray, the base moved to the goal position")
        else:
            rosinfo("The base failed to move to the goal position for some reason")
