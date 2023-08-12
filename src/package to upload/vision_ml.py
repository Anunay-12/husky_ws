#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import pyrealsense2 as rs
import cv2 as cv
import numpy as np
from ultralytics import YOLO
from ultralytics.yolo.v8.detect.predict import DetectionPredictor

model = YOLO('/home/administrator/Models/Trees_v8s_seg.pt')

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

def main():
    rospy.init_node('vision_node')
    pub = rospy.Publisher('/coordinates', Float64MultiArray, queue_size=10)
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
    while not rospy.is_shutdown():
        frame = pipeline.wait_for_frames()
        color_get = frame.get_color_frame()
        depth_get = frame.get_depth_frame()
        color_data = np.asanyarray(color_get.get_data())
        if not color_get:
            continue
        image = cv.cvtColor(color_data, cv.COLOR_BGR2RGB)
        
        coord = (detect_tree(image))
        depth=0
        count=0
        for i in range(-3,4,1):
            for j in range(-3,4,1):
                if depth_get.as_depth_frame().get_distance(coord[0],coord[1]) !=0:
                    count +=1
                    depth += depth_get.as_depth_frame().get_distance(coord[0],coord[1])
        depth = depth/count
        result = rs.rs2_deproject_pixel_to_point(depth_intrinsics, coord, depth)
        result = [result[0], result[2]]
        message = Float64MultiArray()
        message.data = result
        pub.publish(message)
        rate.sleep()


if __name__ == '__main__':
    main()
