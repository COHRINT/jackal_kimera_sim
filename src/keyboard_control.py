#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your Jackal!
---------------------------
Moving around:
        i    
   j    k    l

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0),
    'j': (0, 1),
    'l': (0, -1),
    'k': (-1, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1),
    'x': (0.9, 1),
    'e': (1, 1.1),
    'c': (1, 0.9),
}

# Initialize the ROS node and publisher
rospy.init_node('jackal_teleop')
pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
twist = Twist()

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
	return "currently:\tspeed " + str(speed) + "\tturn " + str(turn)
    # return f"currently:\tspeed {speed}\tturn {turn}"

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    print(msg)
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    x = 0
    th = 0
    status = 0

    try:
        while True:
            key = getKey()

            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0
                th = 0
                if (key == '\x03'):
                    break

            twist.linear.x = x * speed
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = th * turn

            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
