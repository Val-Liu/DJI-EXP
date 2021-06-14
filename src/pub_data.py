#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import math
import numpy as np
import roslib
import rospy
import sys
import time
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image, CameraInfo, NavSatFix, PointCloud2,CompressedImage
from std_msgs.msg import Bool, Empty
from std_msgs.msg import Int8


class collect_data:
    def __init__(self):

        # video width and height
        self.image_pub = rospy.Publisher("/camera/rgb/image_raw", Image, queue_size = 1)
        self.bridge = CvBridge()
        

    def pub_image(self, img):
      #img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
      try:
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
      except CvBridgeError as e:
        print(e)

def main():
    rospy.init_node('collect_data', anonymous=True)
    ss_n = collect_data()
    cap = cv2.VideoCapture('0819I.avi')
    while(cap.isOpened()):
      # Capture frame-by-frame
      ret, frame = cap.read()
      if ret == True:
        ss_n.pub_image(frame)
        # Press Q on keyboard to  exit
        cv2.imshow("1",frame)
        cv2.waitKey(50)
 
      # Break the loop
      else: 
        break
 
    # When everything done, release the video capture object
    cap.release()
 
    # Closes all the frames
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
