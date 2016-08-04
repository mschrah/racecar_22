#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class safety():
	
	def __init__(self):
		self.reverse = AckermannDriveStamped()
		self.reverse.header.stamp = rospy.Time.now()
		self.reverse.drive.speed = -0.5
		'''
		self.forward = AckermannDriveStamped()
		self.forward.header.stamp = rospy.Time.now()
		self.forward.drive.speed = 1
                '''
		#CHANGE
		rospy.Subscriber("/scan", LaserScan, self.scan_callback)
		#CHANGE
		self.pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=0)

                self.bounds = 100
                self.lower = 540 - self.bounds
                self.upper = 540 + self.bounds
                
	def scan_callback(self, msg):
	    sample = msg.ranges[self.lower:self.upper]#200 indices
	    mindist = min(sample)
	    rospy.loginfo(mindist)
	    if mindist < 1:
                    index = sample.index(mindist)
                    self.set_angles(index)
                    self.pub.publish(self.reverse)
                    rospy.sleep(1.)

	def set_angles(self, ind):
                rospy.loginfo(ind)
		if ind < 100:#on right side
			self.reverse.drive.steering_angle = -0.5
			#self.forward.drive.steering_angle = 0.5
		else:#on left side
			self.reverse.drive.steering_angle = 0.5
			#self.forward.drive.steering_angle = -0.5


if __name__ == "__main__":
	rospy.init_node('safety')
	safety_controller = safety()
	rospy.spin()

