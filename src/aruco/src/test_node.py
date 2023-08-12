#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int64

def callback(int):
    pub = rospy.Publisher('/vel2', Int64, queue_size = 10)
    print(int)
    pub.publish(int)
    # return int

def main():
    rospy.init_node("test_node")
    # pub = rospy.Publisher('/vel2', Int64, queue_size=10)
    int = rospy.Subscriber('/vel', Int64,callback)
    # pub.publish(int)
    rospy.spin()

if __name__ == '__main__':
    main()