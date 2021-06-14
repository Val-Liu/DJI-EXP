#!/usr/bin/env python
import rospy,roslib
import time,sys
import cv2
import cv_bridge
import time
import threading
import numpy as np
from distutils.util import strtobool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Empty, Bool, Int8, Int32, String,Int16MultiArray, Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import EmptyResponse as EmptySrvResponse
from behave import *




class drone_value:
	def __init__(self):
		
		self.home = [500.0,500.0]
		self.isFlying = False
		self.isConnected = False
		self.areMotorsOn = False
		self.battery = 100
		self.iniyaw = -1
		self.yaw = 0.0
		self.lat = 500.0
		self.lot = 500.0
		self.alt = 0.0
		self.target_h = 5
		self.gimbal_pitch = 0.0  # up 29.2  ~  -90 down
		self.gimbal_yaw = 0.0
		self.gimbal_roll = 0.0
		self.isContinue = True
		self.orb_scale = False
		self.target = [[24.982022191692824,121.57176196575166],[24.98175232126216,121.57199397683145]]
		self.target_x = 0
		self.target_y = 0
		self.index = 0
		self.current_posx = 0
		self.dist_first = True
		self.start_heading = 0
		self.p0 = 0
		self.p1 = 0
		self.brng = 0
		self.dLon = 0
		self.y = 0
		self.x = 0 
		self.current_yaw = 0
		self.current_alt = 0
		self.dist = 0
		self.pose_dist = 0
		self.heading = 0
		self.cnt = 0
		self.alt_count = 0
		self.alt_count2 = 0
		self.z1 = 0
		self.z2 = 0
		self.z3 = 0
		self.z4 = 0
		self.scale = 0.25
		self.ah_flag=False
		self.feature_flag = False
		self.img_count = 0
		self.query_count = 0
		self.safe = 0
		self.right = 0
		self.left = 0
		self.frame_set = 0
		self.img_cnt = 0
		self.time_set = 0
		self.pose_flag = True
		self.can_forward = False
		self.src_pts = np.array([])
		self.matchesMask = []
		self.count = [False,False,False,False]
		self.pixel_value = [[0,385/2,0,660/2],[385/2,385,0,660/2],[0,385/2,660/2,660],[385/2,385,660/2,660]]
		self.MIN_MATCH_COUNT = 8
		self.fuckthat = 0
		self.q_img = np.array([])
		self.query_tag = False
		self.av_x = 0
		self.av_y = 0
		self.fail_times = 0
		self.total_dist = 40
		self.can_land = False
		self.cen_x = 330
		self.cen_y = 385/2

class flight_contorl:
	def move_s(self, twist, limitTime):
		limitTime = limitTime * 1000
		startTime = int(round(time.time()*1000))
		rate = rospy.Rate(10)
		pub_move_forward = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

		while not rospy.is_shutdown():
			connections = pub_move_forward.get_num_connections()
            
			if connections > 0:
				endTime = int(round(time.time()*1000))
				if endTime - startTime < limitTime:
					pub_move_forward.publish(twist)
				else:
					twist.linear.x = 0
					twist.linear.y = 0
					twist.linear.z = 0
					twist.angular.z = 0
					pub_move_forward.publish(Twist())
		            		break
            		rate.sleep()
	
	def ser_gimbal_zero(self):
		rospy.wait_for_service('/flight_commands/gimbal_zero')
		try:
			call1 = rospy.ServiceProxy('/flight_commands/gimbal_zero', EmptySrv)
			resp1 = call1()
			return resp1
		except rospy.ServiceException, e:
			print "Service call failed: %s" % e
	def ser_gimbal_90(self):
		rospy.wait_for_service('/flight_commands/gimbal_90')
		try:
			call1 = rospy.ServiceProxy('/flight_commands/gimbal_90', EmptySrv)
			resp1 = call1()
			return resp1
		except rospy.ServiceException, e:
			print "Service call failed: %s" % e
	
	def ser_takeoff(self):
		rospy.wait_for_service('/flight_commands/takeoff')
		try:
			call1 = rospy.ServiceProxy('/flight_commands/takeoff', EmptySrv)
			resp1 = call1()
			return resp1
		except rospy.ServiceException, e:
			print "Service call failed: %s" % e
	def ser_land(self):
		rospy.wait_for_service('/flight_commands/land')
		try:
			call1 = rospy.ServiceProxy('/flight_commands/land', EmptySrv)
			resp1 = call1()
			return resp1
		except rospy.ServiceException, e:
			print "Service call failed: %s" % e
	def ser_stop(self):
		rospy.wait_for_service('/flight_commands/stop')
		try:
			call1 = rospy.ServiceProxy('/flight_commands/stop', EmptySrv)
			resp1 = call1()
			return resp1
		except rospy.ServiceException, e:
			print "Service call failed: %s" % e
	def ser_start(self):
		rospy.wait_for_service('/flight_commands/start')
		try:
			call1 = rospy.ServiceProxy('/flight_commands/start', EmptySrv)
			resp1 = call1()
			return resp1
		except rospy.ServiceException, e:
			print "Service call failed: %s" % e	
	
class mission_plan:
	def __init__(self):
		self.state = drone_value()
		self.control = flight_contorl()
		self.receive()
	def receive(self):
		status_sub = rospy.Subscriber("/dji/status", String, self._status_cb, queue_size = 5)
 
	def _status_cb(self, msg):
		temp_data = msg.data.split(";")
		#for i in range(0, len(temp_data)):
		self.state.home = [float(temp_data[0].split("=")[1]), float(temp_data[1].split("=")[1])] 
		if self.state.iniyaw == -1:
			self.state.iniyaw = float(temp_data[3].split("=")[1])
		self.state.yaw = float(temp_data[3].split("=")[1])
		self.state.battery = float(temp_data[4].split("=")[1])
		self.state.isConnected = strtobool(temp_data[5].split("=")[1])
		self.state.areMotorsOn = strtobool(temp_data[6].split("=")[1])
		self.state.isFlying = strtobool(temp_data[7].split("=")[1])
		self.state.lat = float(temp_data[8].split("=")[1])
		self.state.lot = float(temp_data[9].split("=")[1])
		if float(temp_data[14].split("=")[1]) != 0.0:
        		self.state.alt = float(temp_data[14].split("=")[1])
		self.state.alt = float(temp_data[10].split("=")[1])
		self.state.gimbal_pitch = float(temp_data[12].split("=")[1])
		self.state.gimbal_yaw = float(temp_data[13].split("=")[1])
		self.state.gimbal_roll = float(temp_data[11].split("=")[1])
		self.state.indoor_height = float(temp_data[14].split("=")[1])




























