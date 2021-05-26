#!/usr/bin/env python

import rospy
import sys

from std_msgs.msg import String, UInt16MultiArray

class subsBox:
	def __init__(self):
		self.Subscriber = rospy.Subscriber("/boxSize", String, self.callback)
		
		self.List = []
		self.List2 = []
		self.X1 = 0
		self.X2 = 0
		self.X3 = 0

	def callback(self, data):
		pub = rospy.Publisher('/boxList', UInt16MultiArray, queue_size = 10)	
		Data = data.data

		if Data == 'end':
			pub.publish(UInt16MultiArray(data=self.List))

		else:
			area = int(Data)//1000

			if area >= 11:
				self.X1 += 1

			elif area >= 5 and area <= 7:
				self.X2 += 1

			elif area >= 2 and area <= 3:
				self.X3 += 1
				
			self.List = [self.X1, self.X2, self.X3]
		
		print(self.List)

def main(args):
	ic = subsBox()
	rospy.init_node('subsBox', anonymous=True)

	try:
		rospy.spin()

	except KeyboardInterrupt:
		print("shutting down ROS Image feature detector module")

if __name__ == '__main__':
	main(sys.argv)


