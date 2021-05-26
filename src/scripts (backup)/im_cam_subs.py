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

	hsvGap = 14
	lowerBound = np.array([15-hsvGap, 18-hsvGap, 147-hsvGap])
	upperBound = np.array([15+hsvGap, 18+hsvGap, 147+hsvGap])
	
	blur = cv2.GaussianBlur(image_np, ksize=(3,3), sigmaX=0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
	color_mask = cv2.inRange(hsv, lowerBound, upperBound)

	ret1, thr = cv2.threshold(color_mask , 127, 255, 0)
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


        cv2.imshow('color_bitwise', color_mask)
	cv2.imshow('cam_load', image_np)



        key = cv2.waitKey(1) & 0xFF
        if (key == 27): 
            pub.publish("{}".format(time1))
        
  

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
