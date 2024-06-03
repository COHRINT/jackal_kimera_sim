#!/usr/bin/env python
import sys
import numpy as np
import rospy
from geometry_msgs.msg import Twist

class Circle:
        def __init__(self, name):
            self.name = name

            self.pub = rospy.Publisher(name+"/jackal_velocity_controller/cmd_vel",Twist,queue_size=10)
            noise = np.random.normal(0,0.5)
            self.linear_speed = 0.7
            # self.diameter = 2
            # self.angular_speed = -1 * (np.log(self.diameter / 3.634) / -3.036)
            self.angular_speed = -0.3
            rate = rospy.Rate(10)

            while not rospy.is_shutdown():
                t = Twist()
                t.linear.x = self.linear_speed
                t.angular.z = self.angular_speed
                self.pub.publish(t)
                rate.sleep()

if __name__ == '__main__':
    name = sys.argv[1]
    rospy.init_node("circle_"+name)
    c = Circle(name)