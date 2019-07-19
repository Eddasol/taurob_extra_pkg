#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String


n = 230
distance_threshold = 1.2
max_speed_threshold = 2.5
max_speed = 1
min_speed = 0.5


def set_speed(distance):
	velocity = Twist()

	if distance > distance_threshold:
		x = min(max_speed, (max_speed-min_speed)/(max_speed_threshold-distance_threshold)*distance)
		velocity.linear.x = x
	else:
		velocity.angular.z = 0.8

	speed_pub.publish(velocity)


def read_laser(data):
	front_read = [data.ranges[i] for i in range (n, len(data.ranges) - n)]
	set_speed(min(front_read))		 


def main():
    global speed_pub
    rospy.init_node('recovery')
    rospy.Subscriber("taurob_tracker/laser_scan", LaserScan , read_laser)
    speed_pub = rospy.Publisher("cmd_vel_override", Twist)
    rospy.spin()


if __name__ == '__main__':
    main()
