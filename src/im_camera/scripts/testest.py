#!/usr/bin/env python

import rospy
import sys

from std_msgs.msg import UInt16MultiArray

def main():
	pub = rospy.Publisher('/box_position', UInt16MultiArray, queue_size=10)
	rospy.init_node('testest')
	r = rospy.Rate(10)
	
	listData = [[1, 8, 4], [1, 8, 12], 
			[2, 8, 4], [2, 8, 12],
			[3, 8, 4], [3, 8, 12], [100]]

	listData.reverse()

	while not rospy.is_shutdown():
		if len(listData) != 0:
			print(listData)

			pubData = listData.pop()
		
			pub.publish(UInt16MultiArray(data=pubData))
			r.sleep
			
if __name__=='__main__':
	main()
