#!/usr/bin/env python

import rospy
import sys

from std_msgs.msg import Int16MultiArray

class pubArduino:
	def __init__(self):
		self.Subscriber = rospy.Subscriber("/boxPosition", Int16MultiArray, self.callback)

	def callback(self, data):
		pass

class getPosition:
	def __init__(self):
		self.Subscriber = rospy.Subscriber("/boxPosition", Int16MultiArray, self.callback)
		self.boxPosition = []

	def callback(self, data):
		self.boxPosition = data.data
	
		print(self.boxPosition)

def main(args):
	ic = getPosition()
	rospy.init_node('pubArduino', anonymous=True)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("shutting")

if __name__== "__main__":
	main(sys.argv)

