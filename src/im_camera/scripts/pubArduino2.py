#!/usr/bin/env python

import rospy
import sys

from std_msgs.msg import UInt16MultiArray, UInt16, Empty

from copy import copy

class pubArduino:
	def __init__(self):
		self.Subscriber = rospy.Subscriber("test__", Empty, self.callback)
		self.position = []
		self.direction = 0
	
		self.readyState = 0
		
		self.pub = rospy.Publisher("test", UInt16MultiArray, queue_size = 1)

	def printPosition(self):
		print(self.position)
		print("=================")

	def printDirection(self):
		print(self.direction)
		print("=================")

	def callback(self, data):
		
		if self.readyState == 0:
			print("waiting position")

		elif self.readyState == 1:
			self.pub.publish(UInt16MultiArray(data=[100]))
			print("ready ")
			self.readyState = 2

		elif self.readyState == 2:	
			while len(self.position) != 0:
				armPosition = self.position.pop()
				armPosition.append(self.direction)
				
				self.pub.publish(UInt16MultiArray(data=armPosition ))
				print("=================")
				print("arm_position : {}".format(armPosition))	
				print("=================")
				print("left : {}".format(self.position))

class getPosition:
	def __init__(self, pubArduino):
		self.Subscriber = rospy.Subscriber("/box_position", UInt16MultiArray, self.callback)
		self.pubArduino = pubArduino

		self.boxPosition = []

	def callback(self, data):
		Data =  data.data
		if Data[0] == 100:
			self.boxPosition.append(list(Data))

			self.boxPosition.reverse()
			self.pubArduino.position = self.boxPosition
			self.pubArduino.printPosition()
			
			print("=================")
			print("position ready")
			print("=================")
			self.pubArduino.readyState = 1
		else:	
			self.boxPosition.append(list(Data))

	def returnPosition(self):
		return self.boxPosition

class getDirection:
	def __init__(self, pubArduino):
		self.Subscriber = rospy.Subscriber("/box_direction", UInt16, self.callback)
		self.pubArduino = pubArduino
		self.Data = 0

	def callback(self, data):
		self.Data = data.data
		self.pubArduino.direction = self.Data
		self.pubArduino.pritnDirection()
		


def main(args):
	pa = pubArduino()
	gd = getDirection(pa)
	gp = getPosition(pa)

	rospy.init_node('pubArduino', anonymous=True)
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("shutting")

if __name__== "__main__":
	main(sys.argv)

