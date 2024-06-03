#!/usr/bin/env python
"""
	This file saves truth and estimate data at same rate for the euroc dataset.
	Start this before running the bag file, then once you want to save data publish
	message to /save topic.
"""
import rospy
# from geometry_msgs.msg import TransformStamped
# from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import numpy as np
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Path
from multi_slam_and_tracking_ros.srv import request_factors

TEST_NAME = "data/LIO_SAM_SIM_multi_long"


class DataCollector:
	def __init__(self):
		self.sim_data = dict()
		self.sim_data['kipp'] = []
		self.sim_data['tars'] = []

		self.truth_data = dict()
		self.truth_data['kipp'] = []
		self.truth_data['tars'] = []

		self.single_truth = None
		
		rospy.Subscriber("/gazebo/model_states", ModelStates, callback=self.truth_callback)
		rospy.Subscriber("/tars/lio_sam/mapping/path", Path, callback=self.tars_sim_callback)
		rospy.Subscriber("/kipp/lio_sam/mapping/path", Path, callback=self.kipp_sim_callback)
		rospy.Subscriber("/save", Bool, callback=self.save_callback)


	def truth_callback(self, msg):
		self.single_truth = msg
		
	
	def tars_sim_callback(self, msg):

		self.sim_data['tars'].append([msg.poses[-1].pose.position.x,
								msg.poses[-1].pose.position.y,
								msg.poses[-1].pose.position.z])
		
		tars_ind = self.single_truth.name.index('tars')
		
		self.truth_data['tars'].append([self.single_truth.pose[tars_ind].position.x,
								self.single_truth.pose[tars_ind].position.y,
								self.single_truth.pose[tars_ind].position.z])
		rospy.wait_for_service('tars/lio_sam/request_factors')  # Replace 'your_service_name' with the actual service name
		res = rospy.ServiceProxy('tars/lio_sam/request_factors', request_factors)  # Replace 'your_service_name' and 'YourService' with your actual service name and type
		try:
			print(res.infMat)
		except:
			1


	def kipp_sim_callback(self, msg):

		self.sim_data['kipp'].append([msg.poses[-1].pose.position.x,
								msg.poses[-1].pose.position.y,
								msg.poses[-1].pose.position.z])
		
		kipp_ind = self.single_truth.name.index('kipp')
		

		self.truth_data['kipp'].append([self.single_truth.pose[kipp_ind].position.x,
								self.single_truth.pose[kipp_ind].position.y,
								self.single_truth.pose[kipp_ind].position.z])
		

	def save_callback(self, msg):
		print("saving data")
		with open(TEST_NAME + '_truth.npy', 'wb') as f:
			np.save(f, self.truth_data)

		with open(TEST_NAME + '_LIO.npy', 'wb') as f:
			np.save(f, self.sim_data)

		print('--------data saved----------')


if __name__ == "__main__":
	dc = DataCollector()

	rospy.init_node("data_saver")
	rospy.spin()
