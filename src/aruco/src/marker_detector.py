#!/usr/bin/env python3
import rospy
# from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Image, CameraInfo
import cv2 as cv
import math as m
import pyrealsense2 as rs
import numpy as np
from ultralytics import YOLO
from ultralytics.yolo.v8.detect.predict import DetectionPredictor

model = YOLO('/home/Models/Trees_v8n_seg.pt')

def detect_tree(color_frame):
    # cw = color_frame.shape[1] / 2
    # ch = color_frame.shape[0] / 2
    coord = [0,0]
    results = model.predict(color_frame, show=False)
    for result in results:
        boxes = result.boxes
        boxez = boxes.xywh
        boxez = boxez.cpu().numpy()
        for box in boxez:
            x, y, w, h = box
            if x is not None: 
                coord = [int(x),int(y)]
    return coord    


def velocity_control(coord, distance_prev_error, angle_prev_error, distance_error_sum, angle_error_sum):
    # defining pid constants for linear and angular 
    kp_linear = 0.9
    kd_linear = 0.05
    ki_linear = 0

    kp_angular = 1
    kd_angular = 0.1 
    ki_angular = 0
    # defining initial errors
    
    vector = [coord[0], coord[1]]
    distance_error= m.sqrt(coord[0]*coord[0] + coord[1]*coord[1])
    angle_error = m.atan2(coord[0], coord[1])

    linear_vel = 0.2*(kp_linear*distance_error + kd_linear*(distance_error- distance_prev_error) + ki_linear*(distance_error_sum))
    angle_vel = -1*(kp_angular*angle_error + kd_angular*(angle_error - angle_prev_error) + ki_angular*angle_error_sum)
    distance_prev_error = distance_error
    angle_prev_error = angle_error
    distance_error_sum += distance_error
    angle_error_sum += angle_error

    return [linear_vel, angle_vel, distance_prev_error, angle_prev_error, distance_error_sum, angle_error_sum]


def coord_maker(corner,id):
    average = [0,0]
    if len(corner)>0:
        id = id.flatten()
        i=0
        for marker_corner,marker_id in zip(corner,id):
            corner = marker_corner.reshape((4,2))
            (topleft, topright, bottomright, bottomleft) = corner
            average_x = int(int(topleft[0]+topright[0]+bottomleft[0]+bottomright[0])/4)
            average_y = int(int(topleft[1]+topright[1]+bottomleft[1]+bottomright[1])/4)
            average =[average_x,average_y] 
            i+=1
    return average    

def main():
    rospy.init_node('marker_pose_node')
    pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
    rospy.loginfo('the node is working')
    rate = rospy.Rate(10)
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    pipe=pipeline.start(config)

    profile = pipeline.get_active_profile()
    # print(profile)
    depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
    # print(depth_profile)
    depth_intrinsics = depth_profile.get_intrinsics()
    # print(depth_intrinsics)

    distance_prev_error=0
    angle_prev_error=0
    distance_error_sum=0
    angle_error_sum=0
    
    while not rospy.is_shutdown():
        frame = pipeline.wait_for_frames()
        color_get = frame.get_color_frame()
        depth_get = frame.get_depth_frame()
        color_data = np.asanyarray(color_get.get_data())
        if not color_get:
            continue
        image = cv.cvtColor(color_data, cv.COLOR_BGR2RGB)
        
        coord = (detect_tree(image))
        depth = depth_get.as_depth_frame().get_distance(coord[0],coord[1])
        result = rs.rs2_deproject_pixel_to_point(depth_intrinsics, coord, depth)
        result = [result[0], result[2]]
        distance = m.sqrt(result[0]**2 + result[1]**2)
        velocity = [0,0,0,0,0]
        if distance > 0.8:
            velocity = velocity_control(result, distance_prev_error, angle_prev_error, distance_error_sum, angle_error_sum)
            velocity[2] = distance_prev_error
            velocity[3] = angle_prev_error
            velocity[4] = distance_error_sum
            velocity[5] = angle_error_sum
        
        
        print(velocity)
        vel = Twist()
        vel.linear.x = velocity[0]
        vel.angular.z = velocity[1]
        pub.publish(vel)
        cv.waitKey(1)
        rate.sleep()
    
    
    pipeline.stop()


if __name__ == '__main__':
    main()