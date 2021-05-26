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

class im_cam_subs:

    def __init__(self):
        self.subscriber = rospy.Subscriber("/image_raw/compressed", CompressedImage, self.callback, queue_size =1)
        

    def callback(self, ros_data):

		pub = rospy.Publisher('/position', String, queue_size = 0.1)

		np_arr = np.fromstring(ros_data.data, np.uint8)
		image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		image_np2 = image_np.copy()

		blur = cv2.GaussianBlur(image_np, ksize=(3,3), sigmaX=0)
		hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

		rgbMatGap = int(10)
		rgbMatCenterX = 250
		rgbMatCenterY = 250
		
		image_np2 = cv2.circle(image_np2, (rgbMatCenterX, rgbMatCenterY), 1, (0, 0, 255), 2)

		#Hpt1 = (0,rgbMatCenterY)
		#Hpt2 = (640,rgbMatCenterY)
		
		#Vpt1 = (rgbMatCenterX,0)
		#Vpt2 = (rgbMatCenterX,480)
	
		#image_np2 = cv2.line(image_np2, Hpt1, Hpt2, (255,0,0), 2) #hori
		#image_np2 = cv2.line(image_np2, Vpt1, Vpt2, (255,0,0), 2) #verti

			
 
		rgb_box = hsv[rgbMatCenterY-rgbMatGap:rgbMatCenterY+rgbMatGap,rgbMatCenterX-rgbMatGap:rgbMatCenterX+rgbMatGap]
		cv2.imshow('center_color', rgb_box)

		hsvShape = rgb_box[:,:,0].shape
					
		if hsvShape == (20,20):
			hsvHChan = np.reshape(rgb_box[:,:,0],(20*20))
			hsvSChan = np.reshape(rgb_box[:,:,1],(20*20))
			hsvVChan = np.reshape(rgb_box[:,:,2],(20*20))

			
			avgH = sum(hsvHChan)/ len(hsvHChan)
			avgS = sum(hsvSChan)/ len(hsvSChan)
			avgV = sum(hsvVChan)/ len(hsvVChan)			
			
		else:
			pass		
		
		print("H : {}".format(avgH))
		print("S : {}".format(avgS))
		print("V : {}".format(avgV))

		hsvGap = 14
		hsvHGap = avgH-1

		if avgH < hsvGap:
			lower_hsv1 = (0, avgS-hsvGap, avgV-hsvGap)
			upper_hsv1 = (avgH+hsvHGap, avgS+hsvGap, avgV+hsvGap)
			
			lower_hsv2 = (180-(hsvHGap-avgH),avgS-hsvGap, avgV-hsvGap)					
			upper_hsv2 = (180, avgS+hsvGap, avgV+hsvGap)

			img_mask1 = cv2.inRange(hsv, lower_hsv1, upper_hsv1)	
			img_mask2 = cv2.inRange(hsv, lower_hsv2, upper_hsv2)	
			img_mask = img_mask1 + img_mask2

		else :
			lower_hsv = (avgH-hsvHGap, avgS-hsvGap, avgV-hsvGap)
			upper_hsv = (avgH+hsvHGap, avgS+hsvGap, avgV+hsvGap)	

			img_mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
	
	
		#color_mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

		ret1, thr = cv2.threshold(img_mask , 127, 255, 0)
		contours, hierarchy = cv2.findContours(thr, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		if len(contours) > 0:
			for i in range(len(contours)):
				area = cv2.contourArea(contours[i])

				if area > 500:  
					rect = cv2.minAreaRect(contours[i])
					(x, y), (w, h), angle = cv2.minAreaRect(contours[i]) #center position

					box = cv2.boxPoints(rect)
					box = np.int0(box)

					a = int(x)-(int(w)/2) #left edge(a, b) 
					b = int(y)-(int(h)/2)
					c = int(x)+(int(w)/2) #right edge (c, d) 
					d = int(y)+(int(h)/2)

					w = int(w) #width
					h = int(h) #height
					m = w*h

					T1 = (a,b)
					T2 = (c,d)

							   
					time1 = time.strftime('%H:%M:%S',time.localtime(time.time()))

					print(time1)
					print('left edge : {},{}'.format(a,b))
					print('right edge : {},{}'.format(c,d))
					print('width : {}, height : {}'.format(w,h))
					print('area : {}'.format(m))
					print(' ')

					frame = cv2.drawContours(image_np2, [box], -1, (0, 255, 0), 3)

					image_np2 = cv2.circle(image_np2, (box[0][0], box[0][1]), 1, (0, 0, 255), 3)
					cv2.putText(image_np2, str(T1), (box[0][0], box[0][1]), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255), 1)
					print(box)

					
				else: 
					pass

		cv2.imshow('color_bitwise', img_mask)
		cv2.imshow('cam_load', image_np2)



		key = cv2.waitKey(1) & 0xFF
		if (key == 27): 	
			if w >= h:
				boxDirection = 1 #horizontal
			else:
				boxDirection = 2 #vertical
				
			pub.publish("{},{}".format(m, boxDirection))

	def sortBoxPosition(box):
	    box.sort(key=lambda x : x[1])
		boxSorted = box[:]

		for i in range(0,len(box)-1):

			if box[i][1] == box[i+1][1]:
				if box[i][0] > box[i+1][0]:
					boxSorted[i+1] = box[i]
					boxSorted[i] = box[i+1]
				else:
					pass








def main(args):
    ic = im_cam_subs()
    rospy.init_node('im_cam_subs', anonymous=True)
    try:
        rospy.spin()

    except KeyboardInterrupt:
        print "shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ =='__main__':
    main(sys.argv)
