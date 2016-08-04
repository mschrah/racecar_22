#!/usr/bin/env python

#possible delay
import cv2
import rospy
import numpy
from cv_bridge import CvBridge

from racecar_22.msg import BlobDetections
from std_msgs.msg import Float64, String
from sensor_msgs.msg import Image

class Screenshot():
	def __init__ (self):
		rospy.Subscriber('/blob_chal', Image, self.callback)
		rospy.Subscriber('/blob_detection', BlobDetections, self.isBlob)
		self.pub_msg = rospy.Publisher('/exploring_challenge', String, queue_size = 1)

		self.bridge = CvBridge()

		self.seeBlob = ""
		self.count = 0

	def callback(self, msg):
		self.count += 1
		#if self.seeBlob != "":
		img = self.bridge.imgmsg_to_cv2(msg)
		#color = self.seeBlob.data
		
		filename = "/home/racecar/challenge_photos/Image%s.jpg"%(self.count)
		cv2.imwrite(filename,img)
			
		rospy.loginfo("IMAGE IS SAVED")

	def isBlob(self, msg):
		#if there's a blob and the size is above threshold, then we see a blob
		threshold = 0 #0.5 #tbd
		if len(msg.sizes) > 0 and msg.sizes[0] > threshold:
			self.seeBlob = msg.colors[0]
			self.pub_msg.publish(self.seeBlob)

if __name__ == "__main__":
	rospy.init_node('Screenshot')
	s = Screenshot()
	rospy.spin()
