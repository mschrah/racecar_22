#!/usr/bin/env python
	
import cv2
import rospy
from racecar_22.msg import BlobDetections####need to make color field a STRING
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image	
from cv_bridge import CvBridge, CvBridgeError
import threading
import numpy as np
from scipy import misc
	
global colors, color_map
colors = ["red", "green", "yellow", "blue", "pink"]
#color bounds
color_map = {"red": (np.array([0,120,100]), np.array([15,255,255])),\
	"green": (np.array([35,100,200]), np.array([80,255,255])),\
	"yellow": (np.array([20,150,150]), np.array([30,255,255])),\
	"blue": (np.array([110,100,100]), np.array([130,255,255])),\
	"pink": (np.array([146, 30, 75]), np.array ([165, 255, 255]))}
#red sat: 190
# yellow (np.array([20,200,150]), np.array([30,255,255])),\
#blue hue: (np.array([90,100,100]), np.array([150,255,255]))
#	"pink": (np.array([160, 120, 100]), np.array ([180, 255, 255]))}
	
class blob_detector:
	def __init__(self):
		self.node_name = "blob_detector"
		self.thread_lock = threading.Lock()

		self.pub_image = rospy.Publisher("/blob_image", Image, queue_size=1)
		self.pub_chal = rospy.Publisher("/blob_chal", Image, queue_size=1)
		self.pub_msg = rospy.Publisher("/blob_detection", BlobDetections, queue_size=1)
		self.bridge = CvBridge()
	
		rospy.loginfo("[%s] Initialized." %(self.node_name))
	
		self.message = BlobDetections()

		#reference pictures
		self.names = ["ari", "cat", "professor karaman", "racecar"]

		#Initiate SIFT detector
		self.surf = cv2.ORB()

		#function to filter SIFT features
		self.function = lambda d: d.distance < self.threshold
		#good filter size
		self.threshold = 40

		#picture count
		self.piccount = 0

	def cbImage(self,image_msg):
		thread = threading.Thread(target=self.processImage,args=(image_msg,))
		thread.setDaemon(True)
		thread.start()
	
	def processImage(self, image_msg):
		if not self.thread_lock.acquire(False):
			return

		#image setup
		image_cv = self.bridge.imgmsg_to_cv2(image_msg)
		width, height, cha = image_cv.shape
		lowthresarea = 10000#2500

		# Convert BGR to HSV
		hsv = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)

		Arr = []
		BoxArr = []
		cent = Point()

		for c in colors:
			#Create mask
			(lower, upper) = color_map[c]
			mask = cv2.inRange(hsv, lower, upper)
	
			#Create contours
			contours = cv2.findContours(mask, cv2.cv.CV_RETR_EXTERNAL, cv2.cv.CV_CHAIN_APPROX_NONE)[0]
	
			#for each contour of the color
			for co in contours:
				#find the minimum rectangle
				rect = cv2.minAreaRect(co)

				#area
			    	(w,h) = rect[1]
			    	area = w*h
			    	if area < lowthresarea: continue

				#angle = rect[2]
				#if angle < -30 and angle > -60: continue#too angled
				#p = max(w/h,h/w)
				#if p > 2: continue#the rectangle probably does not enclose the paper
				#find the center of the rectangle
				center = rect[0]
				(y,x) = center
				cent.x = x/width
				cent.y = y/height

				#define the box coordinates
				box = cv2.cv.BoxPoints(rect)
				box = np.int0(box)

				#save the appropriate message/image data
				Arr.append((Float64(area),cent,String(c)))
				BoxArr.append(box)#order does not matter
				'''
				if c == "pink":#send image to the correct place
					#cropping picture
					corn1 = box[0]
					corn2 = box[2]
					corn3 = box[1]
					corn4 = box[3]
					p1 = min(min(corn1[1], corn2[1]),min(corn3[1],corn4[1]))
					p2 = max(max(corn1[1], corn2[1]),max(corn3[1],corn4[1]))
					p3 = min(min(corn1[0], corn2[0]),min(corn3[0],corn4[0]))
					p4 = max(max(corn1[0], corn2[0]),max(corn3[0],corn4[0]))
					cropped = image_cv[p1:p2, p3:p4]
					
					#cleaning the image
					img2 = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)			
					if img2 == None: continue		
					widthc, heightc = img2.shape
					
					## find the keypoints and descriptors with SIFT - image
					kp2, des2 = self.surf.detectAndCompute(img2,None)
					if des2 == None: continue #just leave, no keypoints
					flann = cv2.BFMatcher(cv2.NORM_HAMMING,crossCheck=True)

					# Initalize good counts, function
					counts = []

					for name in self.names:
						img1 = misc.imread(name+".jpg","L").astype(np.uint8)
					    
						# find the keypoints and descriptors with SIFT - reference
						kp1, des1 = self.surf.detectAndCompute(img1,None)

						#record number of matches
						matches = flann.match(des1,des2)
						bools = map(self.function,matches)
						count = sum(bools)
						counts.append(count)

					#find max count
					maxi = max(counts)

					#find index of max count
					ind = counts.index(maxi)
					    
					#get picture name of the ind
					chosen = self.names[ind]

					#put string on image
					cv2.putText(cropped, chosen, (int(heightc/2), int(width/2)), cv2.FONT_HERSHEY_PLAIN, 4, (0,255,0))
					#counter for special images
					self.piccount += 1
					#save file as special
					cv2.imwrite("/home/racecar/challenge_photos/special"+self.piccount+".jpg",cropped)
					'''
		#sort blobs
		Arr.sort()
        	Arr = Arr[::-1]
	    	
		#create message
        	self.message.sizes = [o[0] for o in Arr]
        	self.message.locations = [o[1] for o in Arr]
        	self.message.colors = [o[2] for o in Arr]
	
		#sending a challenge picture
		if len(self.message.colors) > 0:		
			cv2.drawContours(image_cv,BoxArr,0,(255,255,255),4)
			colorname = self.message.colors[0].data
			cv2.putText(image_cv, colorname, (100, 100), cv2.FONT_HERSHEY_PLAIN,10, (0,255,0))
			
	
        	try:
			#publish
        		self.pub_image.publish(self.bridge.cv2_to_imgmsg(image_cv, "bgr8"))
        		self.pub_msg.publish(self.message)
			if len(self.message.colors) > 0:
				self.pub_chal.publish(self.bridge.cv2_to_imgmsg(image_cv, "bgr8"))
        	except CvBridgeError as e:
        		print(e)
        	self.thread_lock.release()
	
	
if __name__=="__main__":
	rospy.init_node('blob_detector')
	e = blob_detector()
	sub_image = rospy.Subscriber("/camera/rgb/image_rect_color", Image, e.cbImage, queue_size=1)
	rospy.spin()
