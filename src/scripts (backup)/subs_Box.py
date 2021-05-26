#!/usr/bin/env python

import rospy
import sys

from std_msgs.msg import String

class subs_Box:
	def __init__(self):
		self.Subscriber = rospy.Subscriber("/position", String, self.callback)
		
		self.List = []
		self.List2 = []
		self.X1 = 0
		self.X2 = 0
		self.X3 = 0

	def callback(self, data):
		pub = rospy.Publisher('/box_List', String, queue_size=10)	
		Data = data.data

		if Data.find(',')!=-1:
			listData = list(map(int,Data.split(',')))
			
			area = listData[0]//1000
			dire = listData[1]

			if area >= 11:
				self.X1 += 1
				self.List.append([8, dire])

			elif area >= 5 and area <= 7:
				self.X2 += 1
				self.List.append([8, dire])

			elif area >= 2 and area <= 3:
				self.X3 += 1
				self.List.append([4, dire])
				
			self.List2 = [self.X1, self.X2, self.X3]
		
		else :
			if Data == 'end':
				pub.publish(str(self.List2))
				pass

		print(self.List)
		print(self.List2)

def main(args):
	ic = subs_Box()
	rospy.init_node('subs_Box', anonymous=True)

	try:
		rospy.spin()

	except KeyboardInterrupt:
		print("shutting down ROS Image feature detector module")

if __name__ == '__main__':
	main(sys.argv)


