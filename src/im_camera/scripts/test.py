#!/usr/bin/env python

import rospy
import sys

from std_msgs.msg import UInt16MultiArray

def talker():
	pub = rospy.Publisher('/test', UInt16MultiArray, queue_size = 10)	
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10)


	while not rospy.is_shutdown():
		array = [15,15,12]
		hello_str = UInt16MultiArray(data=array)

		pub.publish(hello_str)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass



