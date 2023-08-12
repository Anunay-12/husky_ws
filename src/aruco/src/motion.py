#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray

def callback(arr):
    pass

if __name__ == '__main__':
    rospy.init_node('motion')
    rospy.Subscriber('/coordinates', Float64MultiArray, callback)
    rospy.spin()