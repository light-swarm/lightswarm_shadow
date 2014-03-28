#!/usr/bin/env python

import rospy
from lightswarm_core.msg import Objects
from lightswarm_core.msg import Cylinder

class ObjectDetector(object):
	def __init__(self):
		rospy.init_node('object_detector')
		self.pub = rospy.Publisher('/objects', Objects)

	def run(self):
		fake_cylinder = Cylinder()
		fake_cylinder.location.x = 0
		fake_cylinder.location.y = 0
		fake_cylinder.height = 180
		fake_cylinder.radius = 20

		objects = Objects()
		objects.cylinders.append(fake_cylinder)

		r = rospy.Rate(10) # 10hz

		while not rospy.is_shutdown():
			#rospy.loginfo('published something')
			self.pub.publish(objects)
			r.sleep()

if __name__ == '__main__':
	detector = ObjectDetector()
	detector.run()

