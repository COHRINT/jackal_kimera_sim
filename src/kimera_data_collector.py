#!/usr/bin/env python
"""
	This file saves truth and estimate data at same rate for the euroc dataset.
	Start this before running the bag file, then once you want to save data publish
	message to /save topic.
"""
import rospy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import numpy as np

TEST_NAME = "data/TYCHO_TEST_DYNAMIC_BAG"
SIM = True


class DataCollector:
	def __init__(self):
		self.kimera_data = []
		self.truth_data = []
		self.single_truth = None

		if SIM:
			rospy.Subscriber("/tars/odometry/global_filtered", Odometry, callback=self.truth_callback)
		else:
			rospy.Subscriber("/vicon/firefly_sbx/firefly_sbx", TransformStamped, callback=self.truth_callback)
		rospy.Subscriber("/kimera_vio_ros/odometry", Odometry, callback=self.kimera_callback)
		rospy.Subscriber("/save", Bool, callback=self.save_callback)


	def truth_callback(self, msg):
		self.single_truth = msg
		
	
	def kimera_callback(self, msg):
		self.kimera_data.append([msg.pose.pose.position.x,
								msg.pose.pose.position.y,
								msg.pose.pose.position.z])
		if SIM:
			self.truth_data.append([self.single_truth.pose.pose.position.x,
									self.single_truth.pose.pose.position.y,
									self.single_truth.pose.pose.position.z])
		else:
			self.truth_data.append([self.single_truth.transform.translation.x,
									self.single_truth.transform.translation.y,
									self.single_truth.transform.translation.z])

	def save_callback(self, msg):
		print("saving data")
		with open(TEST_NAME + '_truth.npy', 'wb') as f:
			np.save(f, self.truth_data)

		with open(TEST_NAME + '_kimera.npy', 'wb') as f:
			np.save(f, self.kimera_data)

		print('--------data saved----------')


if __name__ == "__main__":
	dc = DataCollector()

	rospy.init_node("data_saver")
	rospy.spin()
