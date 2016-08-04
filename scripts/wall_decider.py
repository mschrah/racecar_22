#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped
from racecar_22.msg import BlobDetections

class wall_decider():
	def __init__(self):
		self.pub = rospy.Publisher("/wall", Bool, queue_size=1)
		rospy.Subscriber("/blob_detections", BlobDetections, self.determine_wall)

	def determine_wall(self, msg):
		if len(msg.colors) > 0:
			color = msg.colors[0]
			if color == "red":
				rospy.loginfo("left wall")
				self.pub.publish(True)
			
	#returns index of max value in array
	def find_max(self, array):
		max_val = array[0]
		index = 0
		for i in len(array):
			if array[i] > max_val:
				index = i
				max_val = array[i]
		return index

if __name__ == "__main__":
	rospy.init_node('wall_decider')
	decider = wall_decider()
	rospy.spin()
