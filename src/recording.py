#!/usr/bin/env python2
import cv2
import rospy,roslib
import time
import sys
import numpy as np
from cv_bridge import CvBridge,CvBridgeError 
from sensor_msgs.msg import Image, CompressedImage

global out
fourcc = cv2.VideoWriter_fourcc('X','V','I','D')
out = cv2.VideoWriter('record.avi',fourcc,30.0,(660,385))
class camera_data:
	def __init__(self):
		self.image_np = np.array([])
		self.image_pub = rospy.Publisher("/camera/rgb/image_raw", Image, queue_size = 1)
        	self.bridge = CvBridge()
	def call(self,msg):
		#np_arr = np.fromstring(msg.data, np.uint8)
		#self.image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		self.image_np = CvBridge().imgmsg_to_cv2(msg,"bgr8") #read image
		out.write(self.image_np)
		#cv2.imshow('show',self.image_np)
           	#self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.image_np, "bgr8"))
		cv2.waitKey(1)



	def main(self):
		rospy.init_node('DJI_video')
		#rospy.Subscriber("/image/compressed", CompressedImage, self.call, queue_size = 1)
		self.img_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.call, queue_size = 1)#read image
		rospy.spin()


if __name__ == '__main__':
    try:
	n = camera_data()
    	n.main()
	
    except:
	pass
    finally:
	cv2.destroyAllWindows()
