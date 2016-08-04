#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class trapezoid():
	def __init__(self):
		rospy.Subscriber("/speed_filter", AckermannDriveStamped,self.speed_callback)
		self.pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 1)
		self.prev_speed = 0

	def speed_callback(self, msg):
		self.new_msg = msg
		if abs(self.prev_speed - msg.drive.speed > 0.7):
			if (self.prev_speed == 0 or np.sign(self.prev_speed) == np.sign(msg.drive.speed)):
				if (msg.drive.speed > 0):
					self.prev_speed = self.prev_speed + 0.5
					self.new_msg.drive.speed = self.prev_speed

				elif (msg.drive.speed < 0):
					self.prev_speed = self.prev_speed - 0.5
					self.new_msg.drive.speed = self.prev_speed
			elif (np.sign(self.prev_speed) != np.sign(msg.drive.speed)):
				if (self.prev_speed > 0):
					self.prev_speed = self.prev_speed + 0.5
					self.new_msg.drive.speed = self.prev_speed
				elif (self.prev_speed < 0):
					self.prev_speed = self.prev_speed - 0.5
					self.new_msg.drive.speed = self.prev_speed
		self.pub.publish(self.new_msg)

if __name__ == "__main__":
	rospy.init_node("trapezoid")
	trap = trapezoid()
	rospy.spin()
