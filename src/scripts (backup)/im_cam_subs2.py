#!/usr/bin/env python

# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

class im_cam_subs2:

	def __init__(self):
		self.subscriber = rospy.Subscriber("/image_raw/compressed", CompressedImage, self.callback, queue_size =1)
		#self.pub = rospy.Publisher('/position', String, queue_size = 1)

	def callback(self, ros_data):
		pub = rospy.Publisher('/position', String, queue_size = 1)
		np_arr = np.fromstring(ros_data.data, np.uint8)		
		image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) #image_np.shape = (480, 640, 3)

		blur = cv2.GaussianBlur(image_np, ksize=(5,5), sigmaX=0)
		hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV) 
		gray = hsv[:,:,2]
	
		#gray = cv2.cvtColor(image_np, cv2.COLOR_RGB2GRAY)

		thr1 = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 15, 7)
	
		#opening = cv2.morphologyEx(blur, op=cv2.MORPH_OPEN, kernel=np.ones((3,3), np.uint8), iterations=2)
		#thr1 = cv2.adaptiveThreshold(opening, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 15, 2)
		#edge = cv2.Canny(thr1, 100, 200)

		contours, hierarchy = cv2.findContours(thr1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
		if len(contours) > 0:
			for i in range(len(contours)):
				area = cv2.contourArea(contours[i])
		
				if area > 700:  
				#print("real : {}".format(area))
 
					rect = cv2.minAreaRect(contours[i])
					(x, y), (w, h), angle = cv2.minAreaRect(contours[i]) #center position

					box = cv2.boxPoints(rect)
					box = np.int0(box)
					'''
					rgbMatGap = int(20)
					rgbMatCenterX = int(round(x))
					rgbMatCenterY = int(round(y))
					
					#print(rgbMatGap)

					rgb_box = image_np[rgbMatCenterY-rgbMatGap:rgbMatCenterY+rgbMatGap,rgbMatCenterX-rgbMatGap:rgbMatCenterX+rgbMatGap]
					
					cv2.imshow('center_color', rgb_box)

					hsv2 = cv2.cvtColor(rgb_box, cv2.COLOR_BGR2HSV) 
					hsvShape = hsv2[:,:,0].shape
					
					if hsvShape == (40,40):
						hsvHChan = np.reshape(hsv2[:,:,0],(40*40))
						hsvSChan = np.reshape(hsv2[:,:,1],(40*40))
						hsvVChan = np.reshape(hsv2[:,:,2],(40*40))

						
						avgH = sum(hsvHChan)/ len(hsvHChan)
						avgS = sum(hsvSChan)/ len(hsvSChan)
						avgV = sum(hsvVChan)/ len(hsvVChan)			
						
					else:
						pass		
					
					print("H : {}".format(avgH))
					print("S : {}".format(avgS))
					print("V : {}".format(avgV))

					
					lower_hsv = (avgH-3, avgS-10, avgV-10)
					upper_hsv = (avgH+3, avgS+10, avgV+10)
					
					img_mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
					img_result = cv2.bitwise_and(image_np, image_np, mask = img_mask)
					
					cv2.imshow('img_mask', img_mask)
					cv2.imshow('img_color', img_result)
					
					#print(box)

					'''
					'''
					a = int(x)-(int(w)/2) #left edge(a, b) 
					b = int(y)-(int(h)/2)
					c = int(x)+(int(w)/2) #right edge (c, d) 
					d = int(y)+(int(h)/2)

					w = int(w) #width
					h = int(h) #height
					m = w*h

					T1 = (a,b)
					T2 = (c,d)

					time1 = time.strftime('%H %M %S',time.localtime(time.time()))

					print(time1)
					print('left edge : {},{}'.format(a,b))
					print('right edge : {},{}'.format(c,d))
					print('width : {}, height : {}'.format(w,h))
					print('area : {}'.format(m))
					print(' ')

					cv2.putText(image_np, str(T1), (int(a),int(b)), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255), 1)
					cv2.putText(image_np, str(T2), (int(c),int(d)), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255), 1)
					frame = cv2.drawContours(image_np, [box], -1, (0, 255, 0), 3)
					'''


		#cv2.imshow('blur', blur)
		#cv2.imshow('thres', thr1)
		cv2.imshow('cam_load', image_np)

		key = cv2.waitKey(1) & 0xFF

		if (key == 27): 
			pub.publish("im im")
        
def main(args):
	ic = im_cam_subs2()
	rospy.init_node('im_cam_subs', anonymous=True)

	try:
		rospy.spin()

	except KeyboardInterrupt:
		print "shutting down ROS Image feature detector module"
		cv2.destroyAllWindows()

if __name__ =='__main__':
	main(sys.argv)
