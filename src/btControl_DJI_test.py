#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import geopy.distance
import math
import numpy as np
import roslib
import rospy
import pickle
import sys
import time
import tingyi_DJI_drone
import space_mod

from behave import *
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from pprint import pprint
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CompressedImage, Image, CameraInfo, NavSatFix, PointCloud2
from std_msgs.msg import Bool, Empty


(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')

class btControl_mission:

    # common member
    drone = tingyi_DJI_drone.Drone()
    isContinue = True
    space = space_mod.space_mod()

    width = 856
    height = 480
    #title = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    #fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #allout = cv2.VideoWriter(title + '_all.avi', fourcc, 30,(856, 480))
    bridge = CvBridge()

    # Take Off
    take0ff_complete = False

    def __init__(self):
        self.tree = (
            self.NotFinish >> (self.isNotArrival|self.swichStage ) >> self.fixedPoseAndForward 
        )

        try:
          with open('btModel_BS.pkl', 'rb') as f: 
            model = pickle.load(f)
            self.loadData(model)
        except:
          print("First run")
        #self.title = '2020-02-04-DJI_rivertest'
        #self.fourcc = cv2.VideoWriter_fourcc('X',"V",'I','D')
        #self.out = cv2.VideoWriter(self.title + '_video.avi', self.fourcc, 30,(840, 490))
        # sub
        #self.subscriber = rospy.Subscriber("/image/compressed", CompressedImage, self.image_cb,  queue_size = 1, buff_size=2**24)
        #self.pc2_sub = rospy.Subscriber("/orb_slam2_mono/map_points", PointCloud2, self.pc2_cb, queue_size = 1)

    def saveData(self):
      with open('btModel_BS.pkl', 'wb') as f:
        dict = {}
        dict["space"] = btControl_mission.space
        """dict["take0ff_complete"] = btControl_mission.take0ff_complete
        dict["ini_height"] = btControl_mission.ini_height
        dict["ini_width"] = btControl_mission.ini_width
        dict["downH"] = btControl_mission.downH
        dict["direction"] = btControl_mission.direction
        dict["isSetHome"] = btControl_mission.drone.isSetHome
        dict["home"] = btControl_mission.drone.state.home
        dict["isSetYaw"] = btControl_mission.drone.isSetYaw
        dict["iniPose"] = btControl_mission.drone.state.iniPose
        dict["now_pos"] = btControl_mission.now_pos
        dict["now_h"] = btControl_mission.now_h
        dict["dire"] = btControl_mission.dire
        dict["turn_angle"] = btControl_mission.turn_angle
        dict["now_dis"] = btControl_mission.now_dis"""
        print(dict)
        pickle.dump(dict, f)
        #raise KeyboardInterrupt

    def loadData(self, dict):
        btControl_mission.space = dict["space"]
        """btControl_mission.take0ff_complete = dict["take0ff_complete"]
        btControl_mission.ini_height = dict["ini_height"]
        btControl_mission.ini_width = dict["ini_width"]
        btControl_mission.downH = dict["downH"]
        btControl_mission.direction = dict["direction"]
        btControl_mission.drone.isSetHome = dict["isSetHome"]
        btControl_mission.drone.state.home = dict["home"]
        btControl_mission.drone.isSetYaw = dict["isSetYaw"]
        btControl_mission.drone.state.iniPose = dict["iniPose"]
        btControl_mission.now_pos = dict["now_pos"]
        btControl_mission.now_h = dict["now_h"]
        btControl_mission.dire = dict["dire"]
        btControl_mission.turn_angle = dict["turn_angle"]
        btControl_mission.now_dis = dict["now_dis"] """


    #def image_cb(self, ros_data):
    #    np_arr = np.fromstring(ros_data.data, np.uint8)
    #    image_np = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
    #    #self.out.write(image_np)
    #    cv2.imshow('cv_img', image_np)
    #    cv2.waitKey(1)
        

    @condition
    def NotFinish(self):
        print("condition: NotFinish")
        return btControl_mission.space.getIndex() < btControl_mission.space.getMissionLength()

    @condition
    def isNotArrival(self):
        print("condition: isNotArrival")
        print(btControl_mission.drone.state.lat, btControl_mission.drone.state.lot)
        if btControl_mission.drone.state.lat != 500 and btControl_mission.drone.state.lot != 500:
          now = (btControl_mission.drone.state.lat, btControl_mission.drone.state.lot)
          toT = (btControl_mission.space.getMissionP()[0], btControl_mission.space.getMissionP()[1])
          dis = geopy.distance.vincenty(now, toT).m
          print(dis)
          start_time = rospy.get_time()
          while dis <= 1.0 and dis > 0.6:
            t = Twist()
            btControl_mission.drone.flightCrtl.move(t, 0.1)
            now = (btControl_mission.drone.state.lat, btControl_mission.drone.state.lot)
            dis = geopy.distance.vincenty(now, toT).m
            if rospy.get_time() - start_time >= 1:
              print("pass 2s and not reach")
              break
          return not (dis <= 0.6)

    @action
    def swichStage(self):
        print("action: swichStage")
        btControl_mission.space.setIndex(btControl_mission.space.getIndex() + 1)
        t = Twist()
        btControl_mission.drone.flightCrtl.move_s(t, 0.1)

    @action
    def fixedPoseAndForward(self):
      if btControl_mission.space.getIndex() < btControl_mission.space.getMissionLength():
        print("action: fixedPoseAndForward")
        lat1 = btControl_mission.drone.state.lat
        lat2 = btControl_mission.space.getMissionP()[0]
        lot1 = btControl_mission.drone.state.lot
        lot2 = btControl_mission.space.getMissionP()[1]
        to_angle = btControl_mission.space.angleFromCoordinate(lat1,lot1,lat2,lot2)
        if btControl_mission.drone.state.yaw < 0 :
          now_angle = abs(btControl_mission.drone.state.yaw)
        else:  
          now_angle = 360 - btControl_mission.drone.state.yaw
        print(to_angle, now_angle)
                      
        differ_angle = abs(to_angle - now_angle)
        arcDiffer_angle = 360 - abs(differ_angle)
        
        min_differ_angle = min(differ_angle, arcDiffer_angle)
        
        # need turn
        if min_differ_angle > 5:
          
          if min_differ_angle == differ_angle:
            #  left
            if to_angle > now_angle:
              while True:
                if btControl_mission.drone.state.yaw < 0 :
                  temp_now_angle = abs(btControl_mission.drone.state.yaw)
                else:  
                  temp_now_angle = 360 - btControl_mission.drone.state.yaw
                if to_angle - 2 <= temp_now_angle and temp_now_angle <=  to_angle + 2:
                  break
                t = Twist()
                t.angular.z = -0.03
                btControl_mission.drone.flightCrtl.move(t, 0.1)
            # right
            elif now_angle > to_angle:
              while True:
                if btControl_mission.drone.state.yaw < 0 :
                  temp_now_angle = abs(btControl_mission.drone.state.yaw)
                else:  
                  temp_now_angle = 360 - btControl_mission.drone.state.yaw
                if to_angle - 2 <= temp_now_angle and temp_now_angle <=  to_angle + 2:
                  break
                t = Twist()
                t.angular.z = 0.03
                btControl_mission.drone.flightCrtl.move(t, 0.1)
          
          
          elif min_differ_angle == arcDiffer_angle:
            # right
            if to_angle > now_angle:
              while True:
                if btControl_mission.drone.state.yaw < 0 :
                  temp_now_angle = abs(btControl_mission.drone.state.yaw)
                else:  
                  temp_now_angle = 360 - btControl_mission.drone.state.yaw
                if to_angle - 2 <= temp_now_angle and temp_now_angle <=  to_angle + 2:
                  break
                t = Twist()
                t.angular.z = 0.03
                btControl_mission.drone.flightCrtl.move(t, 0.1)
            
            # left
            elif to_angle < now_angle:
              while True:
                if btControl_mission.drone.state.yaw < 0 :
                  temp_now_angle = abs(btControl_mission.drone.state.yaw)
                else:  
                  temp_now_angle = 360 - btControl_mission.drone.state.yaw
                if to_angle - 2 <= temp_now_angle and temp_now_angle <=  to_angle + 2:
                  break
                t = Twist()
                t.angular.z = -0.03
                btControl_mission.drone.flightCrtl.move(t, 0.1)  

        t = Twist()
        t.linear.x = 2.0
        btControl_mission.drone.flightCrtl.move(t, 0.5)
        
    @action
    def land(self):
        print("down to " + str(2.0))
        while btControl_mission.drone.state.alt > 2.5:
          tx = Twist()
          tx.linear.z = -1.0
          btControl_mission.drone.flightCrtl.move(tx,1)
        print("down to " + str(btControl_mission.drone.state.alt))
        btControl_mission.drone.flightCrtl.ser_land()
        
          
    def up_to(self, height):
      print("up to " + str(height))
      while btControl_mission.drone.state.alt < height:
        tx = Twist()
        tx.linear.z = 1.0
        btControl_mission.drone.flightCrtl.move(tx,0.1)
      print("up to " + str(btControl_mission.drone.state.alt))
      tx = Twist()
      btControl_mission.drone.flightCrtl.move_s(tx,0.1)
 
    def run(self):
        while True:
            if btControl_mission.isContinue == False:
                break
            bb = self.tree.blackboard(1)
            state = bb.tick()
            print "state = %s\n" % state
            if btControl_mission.drone.isStop == True:
              exec("f = open(\"123.txt\",\'rb\')")           
            while state == RUNNING:
                state = bb.tick()
                print "state = %s\n" % state
                if btControl_mission.drone.isStop == True:
                  exec("f = open(\"123.txt\",\'rb\')")
            assert state == SUCCESS or state == FAILURE
            t = rospy.get_time()
            while rospy.get_time() - t < 1.0:
              pass

def main():
    rospy.init_node('btControl_mission_flight', anonymous=True)
    print("start...") 
    btCm_n = btControl_mission()
    #btCm_n.tuneCamera()
    #btCm_n.takeoff()
    time.sleep(3)

    
    print("take off...")
    btCm_n.drone.flightCrtl.ser_takeoff()
    
    while btCm_n.drone.state.alt < 1.1:
      pass
      
    btCm_n.up_to(30)
    #btCm_n.drone.flightCrtl.ser_land()    
    
    #print("tune camera angle...")
    #while btCm_n.drone.state.gimbal_pitch > -25:#-90.0: 
    #  btCm_n.drone.flightCrtl.ser_gimbal_down()    
    try:
        btCm_n.run()
        rospy.spin()
    except IOError, KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
        btCm_n.saveData()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
