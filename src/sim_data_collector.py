#!/usr/bin/env python

import rospy
import numpy as np

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from multi_slam_and_tracking_ros.msg import Results

TEST_NAME = "sim_test"

class DataCollector:
    def __init__(self):
        # Create arrays
        self.tars_kimera = []
        self.tars_truth = []
        self.kipp_LIOSAM = []
        self.kipp_truth = []
        self.target_truth = []
        self.results = []

        self.tars_single_truth = None
        self.kipp_single_truth = None
        self.target_single_truth = None

        self.current_time_step = -1

        # Create subscribers
        rospy.Subscriber("/gazebo/model_states", ModelStates, callback=self.truth_callback)
        rospy.Subscriber("/kimera_vio_ros/odometry", Odometry, callback=self.kimera_callback)
        rospy.Subscriber("/kipp/lio_sam/mapping/path", Path, callback=self.LIOSAM_callback)
        rospy.Subscriber("results", Results, callback=self.results_callback)
        rospy.Subscriber("/save", Bool, callback=self.save_callback)


    def truth_callback(self, msg):
        model_names = msg.name
        
        tars_idx = model_names.index("tars")
        kipp_idx = model_names.index("kipp")
        target_idx = model_names.index("tycho_bot_1")

        self.tars_single_truth = msg.pose[tars_idx]
        self.kipp_single_truth = msg.pose[kipp_idx]
        self.target_single_truth = msg.pose[target_idx]

    def kimera_callback(self, msg):
        self.tars_kimera.append([msg.pose.pose.position.x,
								msg.pose.pose.position.y,
								msg.pose.pose.position.z])
        
        self.tars_truth.append([self.tars_single_truth.position.x,
                                self.tars_single_truth.position.y,
                                self.tars_single_truth.position.z])
        
    def LIOSAM_callback(self, msg):
        self.kipp_LIOSAM.append([msg.poses[-1].pose.position.x,
								 msg.poses[-1].pose.position.y,
								 msg.poses[-1].pose.position.z])
        
        self.kipp_truth.append([self.kipp_single_truth.position.x,
                                self.kipp_single_truth.position.y,
                                self.kipp_single_truth.position.z])
        
    def results_callback(self, msg):
        self.results.append(msg)

        if self.current_time_step != msg.TimeStep:
            self.current_time_step = msg.TimeStep

            self.target_truth.append([self.target_single_truth.position.x,
                                      self.target_single_truth.position.y,
                                      self.target_single_truth.position.z])
        
    def save_callback(self, msg):
        print("saving data")
        with open(TEST_NAME + "_tars_truth.csv", "wb") as f:
            np.savetxt(f, self.tars_truth, delimiter =", ", fmt ='% s')
        with open(TEST_NAME + "_kipp_truth.csv", "wb") as f:
            np.savetxt(f, self.kipp_truth, delimiter =", ", fmt ='% s')
        with open(TEST_NAME + "_target_truth.csv", "wb") as f:
            np.savetxt(f, self.target_truth, delimiter =", ", fmt ='% s')
        with open(TEST_NAME + "_tars_kimera.csv", "wb") as f:
            np.savetxt(f, self.tars_kimera, delimiter =", ", fmt ='% s')
        with open(TEST_NAME + "_kipp_LIOSAM.csv", "wb") as f:
            np.savetxt(f, self.kipp_LIOSAM, delimiter =", ", fmt ='% s')
        with open(TEST_NAME + "_results.csv", "wb") as f:
            np.savetxt(f, self.results, delimiter =", ", fmt ='% s')

        print('--------data saved---------')
        
if __name__ == "__main__":
    dc = DataCollector()

    rospy.init_node("data_collector")
    rospy.spin()