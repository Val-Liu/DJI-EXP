#!/usr/bin/env python2
import cv2
import rospy,roslib
import time
import sys
import math
from cv_bridge import CvBridge,CvBridgeError 
from sensor_msgs.msg import Image,PointCloud2,PointField
from geometry_msgs.msg import PoseStamped
from sensor_msgs import point_cloud2

x = []
y = []
z = []

def main():
    rospy.init_node('h264_listener2')
    rospy.Subscriber("/camera/rgb/image_raw", Image, call, buff_size = 2**24, queue_size = 1)
    rospy.Subscriber("/orb_slam2_mono/pose", PoseStamped, orb_pose)
    rospy.spin()

def orb_pose(pose):
	pose_ = pose 
	x = pose_.pose.position.x
	y = pose_.pose.position.y
	z = pose_.pose.position.z
	print("x:%s" %x,"y:%s" %y,"z:%s" %z)
	
	
			
def call(msg):
	try:
    		img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
		cv2.waitKey(1)
    	except CvBridgeError, e:
        	print e

if __name__ == '__main__':
    try:
    	main()
    except:
	pass
    finally:
	cv2.destroyAllWindows()
