#!/usr/bin/env python3

import rospy
import pyrealsense2 as rs
import cv2 as cv
import math as m
# import numpy as np
from std_msgs.msg import Float64MultiArray

def main():
    rospy.init_node('FGM_gap_array_gen')
    rospy.loginfo('FGM_GP_ARRAY_WORKING')
    pub1 = rospy.Publisher('/FGM_DEPTH_ARRAY', Float64MultiArray, queue_size = 10)
    pub2 = rospy.Publisher('/FGM_ANGLE_ARRAY', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(20)
    
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640,480,rs.format.z16,30)
    # config.enable_stream(rs.stream.color, 640,480,rs.format.rgb8,30)
    pipe = pipeline.start(config)

    profile = pipeline.get_active_profile()
    depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
    depth_intrinsics = depth_profile.get_intrinsics()
    while not rospy.is_shutdown():
        array = []
        array1 = []
        frame = pipeline.wait_for_frames()
        depth_get = frame.get_depth_frame()
        for i in range(0,640):
            depth = depth_get.as_depth_frame().get_distance(i,240)
            result = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [i,240], depth)
            array1.append(m.sqrt(result[0]**2 + result[2]**2))
            angle = m.atan2(result[0], result[2])
            array.append(angle)
        # print(len(array))
        # print(len(array1))
        
        msg = Float64MultiArray()
        msg.data = array1
        pub1.publish(msg)
        msg.data = array
        pub2.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()
