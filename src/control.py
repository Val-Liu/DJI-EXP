#!/usr/bin/env python
import rospy,roslib
import time,sys
import cv2
import cv_bridge
import time
import threading
import geopy
import struct
import math 
import numpy as np
import std_msgs.msg 
from geopy.distance import vincenty
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage,PointCloud2,PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped,PoseStamped
from math import sin, cos, atan2, degrees, acos, radians
from cv_bridge import CvBridge
import DJI_drone
import apriltag
from behave import *

rospy.init_node("landing_mission", anonymous=True)
class bt_missions:
	drone = DJI_drone.mission_plan()
	image_np = np.array([[]])
	pose_ = PoseStamped()
	points_ = PointCloud2()
	rate = rospy.Rate(10)
	pc2_pub = rospy.Publisher("point_cloud_red",PointCloud2,queue_size=1)
	def __init__(self):
		self.img_sub = rospy.Subscriber("/image/compressed", CompressedImage, self.call, buff_size = 2**24, queue_size = 1)
		#self.img_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.call, buff_size = 2**24, queue_size = 1)#read image
		self.orb_sub = rospy.Subscriber("/orb_slam2_mono/pose", PoseStamped, self.orb_pose)
		self.point_sub = rospy.Subscriber("/orb_slam2_mono/map_points",PointCloud2,self.point_cb)
		self.tree = self.orb_correction#(self.feature_match >> ((self.bounding >> self.forward) | self.point_correction)) | self.error_recovery
#self.tree = self.tag #search april tag
#(self.feature_match >> (self.get_scale | self.up_scale) >> ((self.bounding >> self.forward) | self.point_correction)) | self.error_recovery
	def call(self,msg):
		np_arr = np.fromstring(msg.data, np.uint8)
		bt_missions.image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		#bt_missions.image_np = CvBridge().imgmsg_to_cv2(msg,"bgr8") #read image
	def orb_pose(self,pose):
		bt_missions.pose_ = pose 
	def point_cb(self,point):
		bt_missions.points_ = point
	
	

	@condition
	def feature_match(self):
		if bt_missions.drone.state.img_cnt == 0:
			print("img1")
			bt_missions.drone.state.q_img = cv2.imread('7.PNG',0)#far
			bt_missions.drone.state.target_x = 440
			bt_missions.drone.state.target_y = 220
		elif bt_missions.drone.state.img_cnt == 1:
			print("img2")
			bt_missions.drone.state.q_img = cv2.imread('18.PNG',0)#middle
			bt_missions.drone.state.target_x = 350
			bt_missions.drone.state.target_y = 245
		elif bt_missions.drone.state.img_cnt == 2:
			print("img3")
			bt_missions.drone.state.can_land = True
			bt_missions.drone.state.q_img = cv2.imread('337.PNG',0)#close
			bt_missions.drone.state.target_x = 390
			bt_missions.drone.state.target_y = 298
			t = rospy.get_time()
			while rospy.get_time() - t <= 4:
				pass
			return True
		bt_missions.drone.state.img_count = bt_missions.drone.state.img_count + 1
		cv2.imwrite('image/'+str(bt_missions.drone.state.img_count) + '.PNG',bt_missions.image_np)
		img2 = cv2.imread('image/'+str(bt_missions.drone.state.img_count) + '.PNG',cv2.IMREAD_GRAYSCALE)
		img2 = img2[385/5:385*3/5,660/4:660*3/4]
		# Initiate SIFT detector
		sift = cv2.xfeatures2d.SIFT_create()
		kp1, des1 = sift.detectAndCompute(bt_missions.drone.state.q_img,None)
		kp2, des2 = sift.detectAndCompute(img2,None)
		FLANN_INDEX_KDTREE = 0
		index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
		search_params = dict(checks = 50)
		# BFMatcher with default params
		flann = cv2.FlannBasedMatcher(index_params, search_params)
		matches = flann.knnMatch(des1,des2,k=2)
		good = []
		good_without_list = []
		for m,n in matches:
			if m.distance < 0.55*n.distance:
				good.append([m])
				good_without_list.append(m)
		#RANSAC
		if len(good)>bt_missions.drone.state.MIN_MATCH_COUNT and len(good) != 0:
			bt_missions.drone.state.src_pts = np.float32([kp1[mat.queryIdx].pt for mat in good_without_list]).reshape(-1,1,2)
			dst_pts = np.float32([kp2[mat.trainIdx].pt for mat in good_without_list]).reshape(-1,1,2)
			M, mask = cv2.findHomography(bt_missions.drone.state.src_pts, dst_pts, cv2.RANSAC,5.0)
			if M is None:
				pass
			else:
				bt_missions.drone.state.matchesMask = mask.ravel().tolist()
				h,w = bt_missions.drone.state.q_img.shape
				pts = np.float32([[0,0],[0,h-1],[w-1,h-1],[w-1,0]]).reshape(-1,1,2)
				#pts = np.array([pts])
				dst = cv2.perspectiveTransform(pts,M)

			list_kp1 = []
			list_kp2 = []
			delta_x = []
			delta_y = []
			bt_missions.drone.state.src_pts = bt_missions.drone.state.src_pts.tolist()
			for j in range(len(bt_missions.drone.state.matchesMask)):
				if bt_missions.drone.state.matchesMask[j] == 1:
					list_kp1.append(bt_missions.drone.state.src_pts[j][0])
			for i in range(len(list_kp1)):
				x = list_kp1[i][0]
				y = list_kp1[i][1]
				delta_x.append(x - bt_missions.drone.state.target_x)
				delta_y.append(y - bt_missions.drone.state.target_y)
			for cnt in delta_x:
				bt_missions.drone.state.av_x = bt_missions.drone.state.av_x + cnt
			bt_missions.drone.state.av_x = bt_missions.drone.state.av_x/len(delta_x)
			
			for cnt in delta_y:
				bt_missions.drone.state.av_y = bt_missions.drone.state.av_y + cnt
			bt_missions.drone.state.av_y = bt_missions.drone.state.av_y/len(delta_y)
			print(bt_missions.drone.state.av_x,bt_missions.drone.state.av_y)
			return True
		else:
			print "Not enough matches are found - %d/%d" % (len(good),bt_missions.drone.state.MIN_MATCH_COUNT)
			bt_missions.drone.state.matchesMask = None
			return False
		

	@condition
	def bounding(self):
		#pose estimation by feature points
		if (bt_missions.drone.state.av_x < 30 and bt_missions.drone.state.av_x >-30) :
			print("safe")
			return True
		else:
			print("need correction")
			return False

	@action
	def point_correction(self):
		#correction
		if bt_missions.drone.state.av_x >30:
			msg = Twist()
			print("to left side")
			msg.linear.y = -0.5
			bt_missions.drone.control.move_s(msg,2)	
		elif bt_missions.drone.state.av_x <-30:
			msg = Twist()
			print("to right side")
			msg.linear.y = 0.5
			bt_missions.drone.control.move_s(msg,2)
	
	@action
	def forward(self):
		if bt_missions.drone.state.img_cnt !=2:
			print("lock-in and forward")
			msg = Twist()
			msg.linear.x = 0.7
			bt_missions.drone.control.move_s(msg,26)
			bt_missions.drone.state.img_cnt  = bt_missions.drone.state.img_cnt +1
		else:
			bt_missions.drone.control.ser_gimbal_90()
			'''t = rospy.get_time()
			while rospy.get_time() - t <= 3:
				print("wait")'''
			detector = apriltag.Detector()
			image = cv2.cvtColor(bt_missions.image_np, cv2.COLOR_BGR2GRAY)
			result = detector.detect(image)
			print(math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) ,math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y))
			while math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) >= 3 or math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) >= 3 or bt_missions.drone.state.alt > 2:
				detector = apriltag.Detector()
				image = cv2.cvtColor(bt_missions.image_np, cv2.COLOR_BGR2GRAY)
				result = detector.detect(image)
				print(math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) ,math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y))
				#right front
				if math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) > 3 and result[0].center[0] - bt_missions.drone.state.cen_x < 0 and math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) > 3 and result[0].center[1] - bt_missions.drone.state.cen_y > 0:
					print("to left back")
					msg = Twist()
					msg.linear.x = -0.1
					msg.linear.y = -0.1
					msg.linear.z = -0.1
					bt_missions.drone.control.move_s(msg,1)
				#right back
				elif math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) > 3 and result[0].center[0] - bt_missions.drone.state.cen_x < 0 and math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) > 3 and result[0].center[1] - bt_missions.drone.state.cen_y < 0:
					print("to left front")
					msg = Twist()
					msg.linear.x = 0.1
					msg.linear.y = -0.1
					msg.linear.z = -0.1
					bt_missions.drone.control.move_s(msg,1)
				#left front
				elif math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) > 3 and result[0].center[0] - bt_missions.drone.state.cen_x > 0 and math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) > 3 and result[0].center[1] - bt_missions.drone.state.cen_y > 0:
					print("to right back")
					msg = Twist()
					msg.linear.x = -0.1
					msg.linear.y = 0.1
					msg.linear.z = -0.1
					bt_missions.drone.control.move_s(msg,1)
				#left back
				elif math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) > 3 and result[0].center[0] - bt_missions.drone.state.cen_x > 0 and math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) > 3 and result[0].center[1] - bt_missions.drone.state.cen_y < 0:
					print("to right front")
					msg = Twist()
					msg.linear.x = 0.1
					msg.linear.y = 0.1
					msg.linear.z = -0.1
					bt_missions.drone.control.move_s(msg,1)
				#right
				elif math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) > 3 and result[0].center[0] - bt_missions.drone.state.cen_x < 0 and math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) <= 3:
					print("to left")
					msg = Twist()
					msg.linear.y = -0.1
					msg.linear.z = -0.1
					bt_missions.drone.control.move_s(msg,1)
				#left
				elif math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) > 3 and result[0].center[0] - bt_missions.drone.state.cen_x > 0 and math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) <= 3:
					print("to right")
					msg = Twist()
					msg.linear.y = 0.1
					msg.linear.z = -0.1
					bt_missions.drone.control.move_s(msg,1)
				#front
				elif math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) > 3 and result[0].center[1] - bt_missions.drone.state.cen_y > 0 and math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) <= 3:
					print("to back")
					msg = Twist()
					msg.linear.x = -0.1
					msg.linear.z = -0.1
					bt_missions.drone.control.move_s(msg,1)
				#back
				elif math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) > 3 and result[0].center[1] - bt_missions.drone.state.cen_y < 0 and math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) <= 3:
					print("to front")
					msg = Twist()
					msg.linear.x = 0.1
					msg.linear.z = -0.1
					bt_missions.drone.control.move_s(msg,1)
			

			detector = apriltag.Detector()
			image = cv2.cvtColor(bt_missions.image_np, cv2.COLOR_BGR2GRAY)
			result = detector.detect(image)
			print(math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) ,math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y))
			while math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) >= 3 or math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) >= 3:
				#right front
				if math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) > 3 and result[0].center[0] - bt_missions.drone.state.cen_x < 0 and math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) > 3 and result[0].center[1] - bt_missions.drone.state.cen_y > 0:
					print("to left back")
					msg = Twist()
					msg.linear.x = -0.1
					msg.linear.y = -0.1
					
					bt_missions.drone.control.move_s(msg,1)
				#right back
				elif math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) > 3 and result[0].center[0] - bt_missions.drone.state.cen_x < 0 and math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) > 3 and result[0].center[1] - bt_missions.drone.state.cen_y < 0:
					print("to left front")
					msg = Twist()
					msg.linear.x = 0.1
					msg.linear.y = -0.1
					
					bt_missions.drone.control.move_s(msg,1)
				#left front
				elif math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) > 3 and result[0].center[0] - bt_missions.drone.state.cen_x > 0 and math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) > 3 and result[0].center[1] - bt_missions.drone.state.cen_y > 0:
					print("to right back")
					msg = Twist()
					msg.linear.x = -0.1
					msg.linear.y = 0.1
					
					bt_missions.drone.control.move_s(msg,1)
				#left back
				elif math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) > 3 and result[0].center[0] - bt_missions.drone.state.cen_x > 0 and math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) > 3 and result[0].center[1] - bt_missions.drone.state.cen_y < 0:
					print("to right front")
					msg = Twist()
					msg.linear.x = 0.1
					msg.linear.y = 0.1
					
					bt_missions.drone.control.move_s(msg,1)
				#right
				elif math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) > 3 and result[0].center[0] - bt_missions.drone.state.cen_x < 0 and math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) <= 3:
					print("to left")
					msg = Twist()
					msg.linear.y = -0.1
					
					bt_missions.drone.control.move_s(msg,1)
				#left
				elif math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) > 3 and result[0].center[0] - bt_missions.drone.state.cen_x > 0 and math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) <= 3:
					print("to right")
					msg = Twist()
					msg.linear.y = 0.1
					
					bt_missions.drone.control.move_s(msg,1)
				#front
				elif math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) > 3 and result[0].center[1] - bt_missions.drone.state.cen_y > 0 and math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) <= 3:
					print("to back")
					msg = Twist()
					msg.linear.x = -0.1
					
					bt_missions.drone.control.move_s(msg,1)
				#back
				elif math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) > 3 and result[0].center[1] - bt_missions.drone.state.cen_y < 0 and math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) <= 3:
					print("to front")
					msg = Twist()
					msg.linear.x = 0.1
					
					bt_missions.drone.control.move_s(msg,1)
			msg = Twist()
			msg.linear.x = 0.8
			bt_missions.drone.control.move_s(msg,4)
			#landing
			bt_missions.drone.control.ser_land()

	@action
	def tag(self):
		bt_missions.drone.control.ser_gimbal_90()
		t = rospy.get_time()
		while rospy.get_time() - t <= 3:
			print("wait")
			detector = apriltag.Detector()
			image = cv2.cvtColor(bt_missions.image_np, cv2.COLOR_BGR2GRAY)
			result = detector.detect(image)
		print(math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) ,math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y))
		while math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) >= 3 or math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) >= 3 or bt_missions.drone.state.alt >= 3:
			if bt_missions.drone.state.alt >= 3:
				detector = apriltag.Detector()
				image = cv2.cvtColor(bt_missions.image_np, cv2.COLOR_BGR2GRAY)
				result = detector.detect(image)
				print(math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) ,math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y))
				#right front
				if math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) > 3 and result[0].center[0] - bt_missions.drone.state.cen_x < 0 and math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) > 3 and result[0].center[1] - bt_missions.drone.state.cen_y > 0:
					print("to left back")
					msg = Twist()
					msg.linear.x = -0.1
					msg.linear.y = -0.1
					msg.linear.z = -0.1
					bt_missions.drone.control.move_s(msg,1)
				#right back
				elif math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) > 3 and result[0].center[0] - bt_missions.drone.state.cen_x < 0 and math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) > 3 and result[0].center[1] - bt_missions.drone.state.cen_y < 0:
					print("to left front")
					msg = Twist()
					msg.linear.x = 0.1
					msg.linear.y = -0.1
					msg.linear.z = -0.1
					bt_missions.drone.control.move_s(msg,1)
				#left front
				elif math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) > 3 and result[0].center[0] - bt_missions.drone.state.cen_x > 0 and math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) > 3 and result[0].center[1] - bt_missions.drone.state.cen_y > 0:
					print("to right back")
					msg = Twist()
					msg.linear.x = -0.1
					msg.linear.y = 0.1
					msg.linear.z = -0.1
					bt_missions.drone.control.move_s(msg,1)
				#left back
				elif math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) > 3 and result[0].center[0] - bt_missions.drone.state.cen_x > 0 and math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) > 3 and result[0].center[1] - bt_missions.drone.state.cen_y < 0:
					print("to right front")
					msg = Twist()
					msg.linear.x = 0.1
					msg.linear.y = 0.1
					msg.linear.z = -0.1
					bt_missions.drone.control.move_s(msg,1)
				#right
				elif math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) > 3 and result[0].center[0] - bt_missions.drone.state.cen_x < 0 and math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) <= 3:
					print("to left")
					msg = Twist()
					msg.linear.y = -0.1
					msg.linear.z = -0.1
					bt_missions.drone.control.move_s(msg,1)
				#left
				elif math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) > 3 and result[0].center[0] - bt_missions.drone.state.cen_x > 0 and math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) <= 3:
					print("to right")
					msg = Twist()
					msg.linear.y = 0.1
					msg.linear.z = -0.1
					bt_missions.drone.control.move_s(msg,1)
				#front
				elif math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) > 3 and result[0].center[1] - bt_missions.drone.state.cen_y > 0 and math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) <= 3:
					print("to back")
					msg = Twist()
					msg.linear.x = -0.1
					msg.linear.z = -0.1
					bt_missions.drone.control.move_s(msg,1)
				#back
				elif math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) > 3 and result[0].center[1] - bt_missions.drone.state.cen_y < 0 and math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) <= 3:
					print("to front")
					msg = Twist()
					msg.linear.x = 0.1
					msg.linear.z = -0.1
					bt_missions.drone.control.move_s(msg,1)
			else:
				detector = apriltag.Detector()
				image = cv2.cvtColor(bt_missions.image_np, cv2.COLOR_BGR2GRAY)
				result = detector.detect(image)
				print(math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) ,math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y))
				#right front
				if math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) > 3 and result[0].center[0] - bt_missions.drone.state.cen_x < 0 and math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) > 3 and result[0].center[1] - bt_missions.drone.state.cen_y > 0:
					print("to left back")
					msg = Twist()
					msg.linear.x = -0.1
					msg.linear.y = -0.1
					
					bt_missions.drone.control.move_s(msg,1)
				#right back
				elif math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) > 3 and result[0].center[0] - bt_missions.drone.state.cen_x < 0 and math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) > 3 and result[0].center[1] - bt_missions.drone.state.cen_y < 0:
					print("to left front")
					msg = Twist()
					msg.linear.x = 0.1
					msg.linear.y = -0.1
					
					bt_missions.drone.control.move_s(msg,1)
				#left front
				elif math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) > 3 and result[0].center[0] - bt_missions.drone.state.cen_x > 0 and math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) > 3 and result[0].center[1] - bt_missions.drone.state.cen_y > 0:
					print("to right back")
					msg = Twist()
					msg.linear.x = -0.1
					msg.linear.y = 0.1
					
					bt_missions.drone.control.move_s(msg,1)
				#left back
				elif math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) > 3 and result[0].center[0] - bt_missions.drone.state.cen_x > 0 and math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) > 3 and result[0].center[1] - bt_missions.drone.state.cen_y < 0:
					print("to right front")
					msg = Twist()
					msg.linear.x = 0.1
					msg.linear.y = 0.1
					
					bt_missions.drone.control.move_s(msg,1)
				#right
				elif math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) > 3 and result[0].center[0] - bt_missions.drone.state.cen_x < 0 and math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) <= 3:
					print("to left")
					msg = Twist()
					msg.linear.y = -0.1
					
					bt_missions.drone.control.move_s(msg,1)
				#left
				elif math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) > 3 and result[0].center[0] - bt_missions.drone.state.cen_x > 0 and math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) <= 3:
					print("to right")
					msg = Twist()
					msg.linear.y = 0.1
					
					bt_missions.drone.control.move_s(msg,1)
				#front
				elif math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) > 3 and result[0].center[1] - bt_missions.drone.state.cen_y > 0 and math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) <= 3:
					print("to back")
					msg = Twist()
					msg.linear.x = -0.1
					
					bt_missions.drone.control.move_s(msg,1)
				#back
				elif math.fabs(result[0].center[1] - bt_missions.drone.state.cen_y) > 3 and result[0].center[1] - bt_missions.drone.state.cen_y < 0 and math.fabs(result[0].center[0] - bt_missions.drone.state.cen_x) <= 3:
					print("to front")
					msg = Twist()
					msg.linear.x = 0.1
					
					bt_missions.drone.control.move_s(msg,1)
		#landing
		bt_missions.drone.control.ser_land()


	@action
	def error_recovery(self):
		for fuckthat in range(4):
			#cv2.imwrite('image/'+str(bt_missions.drone.state.img_count) + '.PNG',bt_missions.image_np)
			img2 = cv2.imread('image/'+str(bt_missions.drone.state.img_count) + '.PNG',cv2.IMREAD_GRAYSCALE)
			img2 = img2[bt_missions.drone.state.pixel_value[fuckthat][0] : bt_missions.drone.state.pixel_value[fuckthat][1] , bt_missions.drone.state.pixel_value[fuckthat][2] : bt_missions.drone.state.pixel_value[fuckthat][3]]
			# Initiate SIFT detector
			sift = cv2.xfeatures2d.SIFT_create()
			# find the keypoints and descriptors with SIFT
			kp1, des1 = sift.detectAndCompute(bt_missions.drone.state.q_img,None)
			kp2, des2 = sift.detectAndCompute(img2,None)
			FLANN_INDEX_KDTREE = 0
			index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
			search_params = dict(checks = 50)
			# BFMatcher with default params
			flann = cv2.FlannBasedMatcher(index_params, search_params)
			matches = flann.knnMatch(des1,des2,k=2)
			good = []
			good_without_list = []
			for m,n in matches:
				if m.distance < 0.55*n.distance:
					good.append([m])
					good_without_list.append(m)
				
			#RANSAC
			if len(good)>bt_missions.drone.state.MIN_MATCH_COUNT and len(good) != 0:
				bt_missions.drone.state.src_pts = np.float32([kp1[mat.queryIdx].pt for mat in good_without_list]).reshape(-1,1,2)
				dst_pts = np.float32([kp2[mat.trainIdx].pt for mat in good_without_list]).reshape(-1,1,2)
				#print(src_pts)
				M, mask = cv2.findHomography(bt_missions.drone.state.src_pts, dst_pts, cv2.RANSAC,5.0)
				if M is None:
					pass
				else:
					bt_missions.drone.state.matchesMask = mask.ravel().tolist()
					#print(bt_missions.drone.state.matchesMask)
					h,w = bt_missions.drone.state.q_img.shape
					pts = np.float32([[0,0],[0,h-1],[w-1,h-1],[w-1,0]]).reshape(-1,1,2)
					dst = cv2.perspectiveTransform(pts,M)
					bt_missions.drone.state.count[fuckthat] = True
			else:
				print "Not enough matches are found - %d/%d" % (len(good),bt_missions.drone.state.MIN_MATCH_COUNT)
				bt_missions.drone.state.matchesMask = None

		print("error recovery")
		
		if bt_missions.drone.state.count[0] == True or  bt_missions.drone.state.count[1] == True:	
			msg = Twist()
			print("to left side")
			msg.linear.y = -0.5
			bt_missions.drone.control.move_s(msg,2)
		elif bt_missions.drone.state.count[2] == True or  bt_missions.drone.state.count[3] == True:
			msg = Twist()
			print("to right side")
			msg.linear.y = 0.5
			bt_missions.drone.control.move_s(msg,2)

	#orb-slam	
	@condition
	def get_scale(self):
		if bt_missions.drone.state.orb_scale == True:
			return True
		else:
			return False
	@action
	def up_scale(self):
		bt_missions.drone.state.alt_count = bt_missions.drone.state.alt
		bt_missions.drone.state.z1 = bt_missions.pose_.pose.position.z 
		msg = Twist()
		print("up for 5m")
		msg.linear.z = 0.5
		bt_missions.drone.control.move_s(msg,10)
		bt_missions.drone.state.alt_count2 = bt_missions.drone.state.alt
		bt_missions.drone.state.z2 = bt_missions.pose_.pose.position.z
		print("z1,z2 : %s,%s" %(bt_missions.drone.state.z1,bt_missions.drone.state.z2))
		bt_missions.drone.state.scale = math.fabs(bt_missions.drone.state.z2 - bt_missions.drone.state.z1) / (bt_missions.drone.state.alt_count2 - bt_missions.drone.state.alt_count)
		print("scale : %s" %bt_missions.drone.state.scale)
		bt_missions.drone.state.orb_scale = True

	@action
	def orb_correction(self):		
		'''print("orb correction")
		bt_missions.drone.control.ser_gimbal_90()
		t = rospy.get_time()
		while rospy.get_time() - t <= 5:
			#print("search")
			pass
		bt_missions.drone.control.ser_gimbal_zero()
		t = rospy.get_time()
		while rospy.get_time() - t <= 5:
			#print("wait")
			pass
		msg = Twist()'''
		gen = 0
		lst = []
		total_points = []
		pose_x = bt_missions.pose_.pose.position.x
		pose_y = bt_missions.pose_.pose.position.y
		pose_z = bt_missions.pose_.pose.position.z
		assert isinstance(bt_missions.points_,PointCloud2)
		gen = point_cloud2.read_points(bt_missions.points_,field_names=("x","y","z"),skip_nans=True)
		time.sleep(1)
		lst = list(gen)
		total_points = [(lst[i][0],lst[i][1],lst[i][2]) for i in range(len(lst)) if 0 < lst[i][0] < pose_x + bt_missions.drone.state.scale*4 and pose_y - bt_missions.drone.state.scale*1< lst[i][1] < pose_y +bt_missions.drone.state.scale*1 and pose_z -bt_missions.drone.state.scale*1< lst[i][2] < pose_z + bt_missions.drone.state.scale*1]
		print("(dist)current point inside the box: %s" %len(total_points))
		if len(total_points) < 200:
			print("Safe.")
			'''msg = Twist()
			msg.linear.x = 0.5
			bt_missions.drone.control.move_s(msg,1)'''
		else:
			print("Obstacle!!")
			#bt_missions.drone.control.ser_land()
		fields = [PointField('x', 0, PointField.FLOAT32, 1),PointField('y', 4, PointField.FLOAT32, 1),PointField('z', 8, PointField.FLOAT32, 1),PointField('rgba', 12, PointField.UINT32, 1)]
		rgb1 = struct.unpack('I', struct.pack('BBBB', 0, 0, 255, 255))[0]
		point_list = []
		for i in total_points:
        		point_list.append((i[0] ,i[1],i[2],rgb1)) 
        	header = std_msgs.msg.Header()
        	header.stamp = rospy.Time.now()
        	header.frame_id = 'map'
        	pcl = point_cloud2.create_cloud(header,fields,point_list)
        	bt_missions.pc2_pub.publish(pcl)
	
	@action
	def landing(self):
		bt_missions.drone.control.ser_land()
		print("land")
		
	def run(self):
		while True:
			if bt_missions.drone.state.isContinue == False:
				break
			bt_count = self.tree.blackboard(1)
			bt_state = bt_count.tick()
			#print("bt state = %s\n" % bt_state)
			while bt_count == RUNNING:
				bt_state = bt_count.tick()
				#print("state = %s\n" %bt_state)
			assert bt_state == SUCCESS or bt_state == FAILURE



			
def main():
	print("start BT:")
	btCm_n = bt_missions()
	t = rospy.get_time()
	while rospy.get_time() - t <= 3:
		pass
	try:
		btCm_n.run()
    	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main()
