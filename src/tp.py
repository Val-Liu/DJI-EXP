#!/usr/bin/env python2
import cv2
import rospy,roslib
import numpy as np
import time
import sys
from cv_bridge import CvBridge,CvBridgeError 
from sensor_msgs.msg import Image



global c
c = 1
def main():
    rospy.init_node('DJI_image)
    rospy.Subscriber("image/compressed", CompressedImage, call, buff_size = 2**24, queue_size = 1)
    rospy.spin()
def call(msg):
	try:
		global c
		time_f = 100
    	np_arr = np.fromstring(msg.data, np.uint8)
		image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		if (c % time_f == 0):
			cv2.imwrite('image/'+str(c) + '.PNG',image_np)
		c = c+1
		cv2.imshow('show',img)
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
    


