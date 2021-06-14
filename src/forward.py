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


rospy.init_node("GPS", anonymous=True)
class forward_GPS:
	drone = DJI_drone.mission_plan()
	image_np = np.array([[]])
	rate = rospy.Rate(10)
	pc2_pub = rospy.Publisher("point_cloud_red",PointCloud2,queue_size=1)
	def __init__(self):
		self.img_sub = rospy.Subscriber("/image/compressed", CompressedImage, self.call, buff_size = 2**24, queue_size = 1)
		#self.img_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.call, buff_size = 2**24, queue_size = 1)#read image
		self.orb_sub = rospy.Subscriber("/orb_slam2_mono/pose", PoseStamped, self.orb_pose)
		self.point_sub = rospy.Subscriber("/orb_slam2_mono/map_points",PointCloud2,self.point_cb)
		self.tree = self.achieve | ((self.ac_alt | self.up) >> (self.yaw| self.yaw_correction) >> (self.forward))
#(self.feature_match >> (self.get_scale | self.up_scale) >> ((self.bounding >> self.forward) | self.point_correction)) | self.error_recovery
	def call(self,msg):
		np_arr = np.fromstring(msg.data, np.uint8)
		forward_GPS.image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		#forward_GPS.image_np = CvBridge().imgmsg_to_cv2(msg,"bgr8") #read image
	def orb_pose(self,pose):
		forward_GPS.pose_ = pose 
	def point_cb(self,point):
		forward_GPS.points_ = point
	
			
	@condition
	def achieve(self):
		if forward_GPS.drone.state.index == 0 :
			#GPS distance
			forward_GPS.drone.control.ser_gimbal_zero()
			forward_GPS.drone.state.p0 = (forward_GPS.drone.state.lat, forward_GPS.drone.state.lot)
			forward_GPS.drone.state.p1 = (forward_GPS.drone.state.target[forward_GPS.drone.state.index][0],forward_GPS.drone.state.target[forward_GPS.drone.state.index][1])
			forward_GPS.drone.state.dist = vincenty(forward_GPS.drone.state.p0,forward_GPS.drone.state.p1).meters
			print("current Gps dist: %s" %forward_GPS.drone.state.dist)
			
			if forward_GPS.drone.state.dist >= 5:
				return False
			else:
				forward_GPS.drone.state.index = forward_GPS.drone.state.index + 1
				return True

		else:
			forward_GPS.drone.state.p0 = (forward_GPS.drone.state.lat, forward_GPS.drone.state.lot)
			forward_GPS.drone.state.p1 = (forward_GPS.drone.state.target[forward_GPS.drone.state.index][0],forward_GPS.drone.state.target[forward_GPS.drone.state.index][1])
			forward_GPS.drone.state.dist = vincenty(forward_GPS.drone.state.p0,forward_GPS.drone.state.p1).meters
			print("current Gps dist: %s" %forward_GPS.drone.state.dist)
			if forward_GPS.drone.state.dist >= 5:
				return True		
			else:
				return True
	@condition
	def ac_alt(self):
		if forward_GPS.drone.state.alt > 30:
			print("altitude enough")
			return True
		else:
			print("altitude not enough")
			return False
	
	@condition
	def yaw(self):
		if forward_GPS.drone.state.index == 0:
			forward_GPS.drone.state.dLon = (forward_GPS.drone.state.target[forward_GPS.drone.state.index][1] - forward_GPS.drone.state.home[1])
			forward_GPS.drone.state.y = math.sin(forward_GPS.drone.state.dLon) * math.cos(forward_GPS.drone.state.target[forward_GPS.drone.state.index][0])
			forward_GPS.drone.state.x = math.cos(forward_GPS.drone.state.home[0]) * math.sin(forward_GPS.drone.state.target[0]) - math.sin(forward_GPS.drone.state.home[0]) * cos(forward_GPS.drone.state.target[forward_GPS.drone.state.index][0]) * math.cos(forward_GPS.drone.state.dLon)
			forward_GPS.drone.state.brng = math.atan2(forward_GPS.drone.state.y, forward_GPS.drone.state.x)
			forward_GPS.drone.state.brng = math.degrees(forward_GPS.drone.state.brng)
			forward_GPS.drone.state.brng = (forward_GPS.drone.state.brng + 360) % 360
			forward_GPS.drone.state.brng = 360 - forward_GPS.drone.state.brng
			forward_GPS.drone.state.current_yaw = forward_GPS.drone.state.yaw
			print("correction heading: %s" %forward_GPS.drone.state.current_yaw)
                        print("correction brng: %s" %forward_GPS.drone.state.brng)
			if forward_GPS.drone.state.current_yaw >(360 - forward_GPS.drone.state.brng) + 2 or forward_GPS.drone.state.current_yaw <(360 - forward_GPS.drone.state.brng) -2:
				return False
			else:
				return True
		else:
			forward_GPS.drone.state.dLon = (forward_GPS.drone.state.target[forward_GPS.drone.state.index][1] - forward_GPS.drone.state.lot)
			forward_GPS.drone.state.y = math.sin(forward_GPS.drone.state.dLon) * math.cos(forward_GPS.drone.state.target[forward_GPS.drone.state.index][0])
			forward_GPS.drone.state.x = math.cos(forward_GPS.drone.state.lat) * math.sin(forward_GPS.drone.state.target[forward_GPS.drone.state.index][0]) - math.sin(forward_GPS.drone.state.lat) * cos(forward_GPS.drone.state.target[forward_GPS.drone.state.index][0]) * math.cos(forward_GPS.drone.state.dLon)
			forward_GPS.drone.state.brng = math.atan2(forward_GPS.drone.state.y,drone.state.x)
			forward_GPS.drone.state.brng = math.degrees(forward_GPS.drone.state.brng)
			forward_GPS.drone.state.brng = (forward_GPS.drone.state.brng + 360) % 360
			forward_GPS.drone.state.brng = 360 - forward_GPS.drone.state.brng
			forward_GPS.drone.state.current_yaw = forward_GPS.drone.state.yaw
			print("correction heading: %s" %forward_GPS.drone.state.current_yaw)
                        print("correction brng: %s" %forward_GPS.drone.state.brng)
			if forward_GPS.drone.state.current_yaw >(360 - forward_GPS.drone.state.brng) + 2 or forward_GPS.drone.state.current_yaw <(360 - forward_GPS.drone.state.brng) -2:
				return False
			else:
				return True
			
	@action
	def yaw_correction(self):
		#forward_GPS.drone.control.ser_gimbal_zero()#camera forward
		msg = Twist()
		#msg.header.stamp = rospy.Time.now()
		forward_GPS.drone.state.current_yaw = forward_GPS.drone.state.yaw
		if 360 - forward_GPS.drone.state.brng - forward_GPS.drone.state.current_yaw > 0 and 360 - forward_GPS.drone.state.brng - forward_GPS.drone.state.current_yaw < 180:
			msg.angular.z = -0.1
		elif 360 - forward_GPS.drone.state.brng - forward_GPS.drone.state.current_yaw > 0 and 360 - forward_GPS.drone.state.brng - forward_GPS.drone.state.current_yaw >= 180:  
			msg.angular.z = 0.1
		elif 360 - forward_GPS.drone.state.brng - forward_GPS.drone.state.current_yaw < 0 and math.fabs(360 - forward_GPS.drone.state.brng - forward_GPS.drone.state.current_yaw) >= 180:
			msg.angular.z = -0.1
		elif 360 - forward_GPS.drone.state.brng - forward_GPS.drone.state.current_yaw < 0 and math.fabs(360 - forward_GPS.drone.state.brng - forward_GPS.drone.state.current_yaw) < 180:
			msg.angular.z = 0.1
		forward_GPS.drone.state.current_yaw = forward_GPS.drone.state.yaw
		while forward_GPS.drone.state.current_yaw >(360 - forward_GPS.drone.state.brng) + 2.5 or forward_GPS.drone.state.current_yaw <(360 - forward_GPS.drone.state.brng) -2.5:
			msg = Twist()
			forward_GPS.drone.control.move_s(msg,5)
			forward_GPS.drone.state.current_yaw = forward_GPS.drone.state.yaw
			print("yaw rotate")
	@action
	def up(self):
		while forward_GPS.drone.state.alt < 30:
			msg = Twist()
			print("up")
			msg.linear.z = 0.5
			bt_missions.drone.control.move_s(msg,2)
	@action
	def forward(self):
		forward_GPS.drone.state.p0 = (forward_GPS.drone.state.lat, forward_GPS.drone.state.lot)
		forward_GPS.drone.state.p1 = (forward_GPS.drone.state.target[forward_GPS.drone.state.index][0],forward_GPS.drone.state.target[forward_GPS.drone.state.index][1])
		forward_GPS.drone.state.dist = vincenty(forward_GPS.drone.state.p0,forward_GPS.drone.state.p1).meters
		print("current Gps dist: %s" %forward_GPS.drone.state.dist)
		while forward_GPS.drone.state.dist >=5:
			msg = Twist()
			msg.linear.x = 0.7
			bt_missions.drone.control.move_s(msg,2)
			forward_GPS.drone.state.p0 = (forward_GPS.drone.state.lat, forward_GPS.drone.state.lot)
			forward_GPS.drone.state.p1 = (forward_GPS.drone.state.target[forward_GPS.drone.state.index][0],forward_GPS.drone.state.target[forward_GPS.drone.state.index][1])
			forward_GPS.drone.state.dist = vincenty(forward_GPS.drone.state.p0,forward_GPS.drone.state.p1).meters
			print("current Gps dist: %s" %forward_GPS.drone.state.dist)

	def run(self):
		while True:
			if forward_GPS.drone.state.isContinue == False:
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
	btCm_n = forward_GPS()
	t = rospy.get_time()
	while rospy.get_time() - t <= 3:
		pass
	try:
		btCm_n.run()
    	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main()
