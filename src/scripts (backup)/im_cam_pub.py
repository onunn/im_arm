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
VERBOSE=False


def im_cam_pub():
    image_pub = rospy.Publisher("/output/image_raw/Compressed", CompressedImage, queue_size = 1)
    rospy.init_node('im_cam_pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_EXPOSURE, 40)

    while(cap.isOpened()):
	ret, frame = cap.read()

	cv2.imshow('sibal', frame)

'''
msg = CompressedImage()
msg.header.stamp = rospy.Time.now()
msg.format = "jpeg"
msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
'''


#self.subscriber.unregister()
'''
def main(args):
    ic = image_feature()
    rospy.init_node('image_feature', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()
'''

if __name__ == '__main__':
    try:
        im_cam_pub()
    except rospy.ROSInterruptException:
        pass
