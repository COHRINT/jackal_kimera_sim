#!/usr/bin/env python

# Think I can delete this file


import rospy
from sensor_msgs.msg import Image, Imu


class Republiser:

	def __init__(self):

		self.img1_pub = rospy.Publisher("/camera/infra1/image_rect_raw", Image, queue_size=10)
		self.img2_pub = rospy.Publisher("/camera/infra2/image_rect_raw", Image, queue_size=10)
		self.imu_pub = rospy.Publisher("/camera/imu", Imu, queue_size=10)

		self.imu_data = None
		self.img2_data = None

		rospy.Subscriber("/camera/infra1/image_raw", Image, callback=self.image1_callback)
		rospy.Subscriber("/camera/infra2/image_raw", Image, callback=self.image2_callback)
		rospy.Subscriber("/imu/data", Imu, callback=self.imu_callback)



	def image1_callback(self, msg):
		if self.img2_data is None or self.imu_data is None:
			return

		
		img2_data = self.img2_data
		img2_data.header = msg.header

		imu_data = self.imu_data
		imu_data.header = msg.header

		self.img1_pub.publish(msg)
		self.img2_pub.publish(img2_data)
		self.imu_pub.publish(imu_data)



	def image2_callback(self, msg):
		self.img2_data = msg

	def imu_callback(self, msg):
		self.imu_data = msg



if __name__ == "__main__":
	rospy.init_node("data_republisher")

	r = Republiser()

	rospy.spin()