#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class safety():
	
	def __init__(self):
		self.reverse = AckermannDriveStamped()
		self.reverse.header.stamp = rospy.Time.now()
		self.reverse.drive.speed = -0.5
		rospy.Subscriber("scan", LaserScan, self.scan_callback)
		self.pub = rospy.Publisher("vesc/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=0)

	def scan_callback(msg):
	    sample = msg.ranges[490:591]
	    for i in range(10,90):
	        if sample[i] < 0.75:
        	    if abs(sum(sample[i+1:i+10])/10 - sum(sample[i-10:i])/10) < 0.1:
			set_reverse_angle(i)
			self.safety_pub.publish(reverse)

	def set_reverse_angle(index):
		if index < 50:
			self.reverse.drive.steering_angle = -0.5
		else:
			self.reverse.drive.steering_angle = 0.5


if __name__ == "__main__":
	rospy.init_node('safety')
	safety_controller = safety()
	rospy.spin()
