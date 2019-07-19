#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Possible topics that can deside the velocity
DEFAULT = "cmd_vel_default"
OVERRIDE = "cmd_vel_override"


# Which of the topics that get to control the velocity at the current time
writer = DEFAULT

# Publisher where the velocity get published
vel_pub = None

def update_state(data):
	# data.data is a string with the topic which should be in control
	global writer
	if writer != data.data:
		print("Writer is updated from: ", writer, "to: ", data.data)

		vel_pub.publish(Twist())

	if data.data == OVERRIDE:
		writer = OVERRIDE
	else:
		writer = DEFAULT


def default_writer(velocity):
	# Pases on the velocity from default velocity topic if it's choosen as the writer
	if writer != DEFAULT:
		return
	
	vel_pub.publish(velocity)


def override_writer(velocity):
	# Pases on the velocity from override velocity topic if it's choosen as the writer
	if writer != OVERRIDE:
		return
	
	vel_pub.publish(velocity)


def main():
    global vel_pub
    rospy.init_node('recovery')
    rospy.Subscriber("navigation_status", String, update_state) 
    rospy.Subscriber(DEFAULT, Twist, default_writer)
    rospy.Subscriber(OVERRIDE, Twist, override_writer)
    vel_pub = rospy.Publisher("taurob_tracker/cmd_vel_raw", Twist, queue_size = 10)
    rospy.spin()


if __name__ == '__main__':
    main()
