#!/usr/bin/env python

import rospy
import numpy as np
import csv

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from multi_slam_and_tracking_ros.msg import Results

TEST_NAME = "data/sim2"

class DataCollector:
    def __init__(self):
        # Create arrays
        self.tars_kimera = []
        # self.tars_kimera.append(["x","y","z"])
        self.tars_truth = []
        # self.tars_truth.append(["x","y","z"])
        self.kipp_LIOSAM = []
        # self.kipp_LIOSAM.append(["x","y","z"])
        self.kipp_truth = []
        # self.kipp_truth.append(["x","y","z"])
        self.target_truth = []
        # self.target_truth.append(["x","y","z"])
        self.results = []
        # self.results.append(["TimeStep","Agent","Target","LambdaMin","TMu","TMuDim","TCov","TCovDim"])

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
        self.results.append([str(msg.TimeStep),
                             str(msg.Agent),
                             str(msg.Target),
                             str(msg.TMu),
                             str(msg.TMuDim),
                             str(msg.TCov),
                             str(msg.TCovDim)])
        # self.results.append(msg)

        if self.current_time_step != msg.TimeStep:
            self.current_time_step = msg.TimeStep

            self.target_truth.append([self.target_single_truth.position.x,
                                      self.target_single_truth.position.y,
                                      self.target_single_truth.position.z])
        
    def save_callback(self, msg):
        print("saving data")
        with open(TEST_NAME + "_tars_truth.csv", "w") as f:
            write = csv.writer(f)
            write.writerow(['x','y','z'])
            write.writerows(self.tars_truth)
        with open(TEST_NAME + "_kipp_truth.csv", "w") as f:
            write = csv.writer(f)
            write.writerow(['x','y','z'])
            write.writerows(self.kipp_truth)
        with open(TEST_NAME + "_target_truth.csv", "w") as f:
            write = csv.writer(f)
            write.writerow(['x','y','z'])
            write.writerows(self.target_truth)
        with open(TEST_NAME + "_tars_kimera.csv", "w") as f:
            write = csv.writer(f)
            write.writerow(['x','y','z'])
            write.writerows(self.tars_kimera)
        with open(TEST_NAME + "_kipp_LIOSAM.csv", "w") as f:
            write = csv.writer(f)
            write.writerow(['x','y','z'])
            write.writerows(self.kipp_LIOSAM)
        with open(TEST_NAME + "_results.csv", "w") as f:
            write = csv.writer(f)
            write.writerow(['TimeStep','Agent','Target','TMu','TMuDim','TCov','TCovDim'])
            write.writerows(self.results)

        print('--------data saved---------')
        
if __name__ == "__main__":
    dc = DataCollector()

    rospy.init_node("data_collector")
    rospy.spin()