#!usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import math as m

count=0
pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size = 10)

def velocity_publisher(coord):
    global pub
    # defining pid constants for linear and angular 
    kp_linear = 1.2
    kd_linear = 0.1
    ki_linear = 0

    kp_angular = 1
    kd_angular = 0.1
    ki_angular = 0
        # defining initial errors
    global distance_prev_error
    global angle_prev_error
    global distance_error_sum
    global angle_error_sum
        
    distance_error= m.sqrt(coord[0]*coord[0] + coord[1]*coord[1])
    angle_error = m.atan2(coord[0], coord[1])

    linear_vel = 0.15*(kp_linear*distance_error + kd_linear*(distance_error- distance_prev_error) + ki_linear*(distance_error_sum))
    angle_vel = -1*(kp_angular*angle_error + kd_angular*(angle_error - angle_prev_error) + ki_angular*angle_error_sum)    
    distance_prev_error = distance_error
    angle_prev_error = angle_error
    distance_error_sum += distance_error
    angle_error_sum += angle_error

    Twist_msg = Twist()
    Twist_msg.linear.x = linear_vel
    Twist_msg.angular.z = angle_vel
    pub.publish(Twist_msg)



def callback(coord):
    coord = coord.data
    global count, pub
    distance = m.sqrt(coord[0]**2 + coord[1]**2)
    if distance < 0.8:
        count +=1
        if count >=50:
            Twist_msg = Twist()
            pub.publish(Twist_msg)
            count =0
    else :
        count =0
        velocity_publisher(coord)


if __name__ =='__main__':
    rospy.init_node('motion_2')
    while not rospy.is_shutdown():
        rospy.Subscriber('/coordinates', Float64MultiArray, callback)
        rospy.spin()