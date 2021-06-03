#!/usr/bin/env python

import rospy
import sys

from std_msgs.msg import UInt16MultiArray, Empty

class talker:
	def __init__(self):
		self.Subscriber = rospy.Subscriber("test__", Empty, self.callback)
		#self.pub = rospy.Publisher('/test', UInt16MultiArray, queue_size = 1)
		self.readyState = 0

	def callback(self, data):
		pub = rospy.Publisher("/test", UInt16MultiArray, queue_size = 1)

		if self.readyState == 0:
			print("waiting position")

		elif self.readyState == 1:
			pub.publish(UInt16MultiArray(data=[100]))
			print("position ready")
			self.readyState = 2

		elif self.readyState == 2:	
			pub.publish(UInt16MultiArray(data=[5,5,2])


class listener:
	def __init__(self, talker):
		self.Subscriber = rospy.Subscriber("/list", UInt16MultiArray, self.callback)
		self.talker = talker

	def callback(self, data):
		Data = data.data

		if Data[0] == 200:
			self.talker.readyState = 1 #position ready
			print("ready")

def main(args):
	rospy.init_node('talker', anonymous = True)

	rc = talker()
	ls = listener(rc)
	try:
		rospy.spin()

	except KeyboardInterrupt:
		print("shutting")


if __name__ == '__main__':
	main(sys.argv)
