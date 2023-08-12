#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int64

def main():
    rospy.init_node("test_node2")
    pub = rospy.Publisher('/vel', Int64, queue_size=10)
    i=0
    while not rospy.is_shutdown():
        pub.publish(i)
        i+=1
        rospy.sleep(1)
    rospy.spin()

if __name__ == '__main__':
    main()