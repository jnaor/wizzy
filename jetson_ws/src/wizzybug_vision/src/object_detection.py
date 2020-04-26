#!/usr/bin/env python

import cv2
import numpy as np
import sys
import os

# import pyrosenv
# from pyrosenv import rospy
# from pyrosenv.sensor_msgs.msg import Image

import rospy
from sensor_msgs.msg import Image

# from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge, CvBridgeError
# from cv_bridge.boost.cv_bridge_boost import getCvType

# sys.path.append(os.path.join(os.getcwd(), 'devel/lib/python2.7/dist-packages'))
# sys.path.append(os.path.join(os.getcwd(), 'devel/lib/python3/dist-packages'))
import wizzybug_msgs.msg as wizzybug_msgs

import segmentation as segmentation


class ImageContainer:
    def __init__(self, cam="camera"):

        # subscribe to depth image
        self.depth_sub = rospy.Subscriber(
            "/{}/depth/image_rect_raw".format(cam), Image, self.depth_callback)

        # subscribe to rgb
        # self.rgb_sub = rospy.Subscriber(
        #     "/{}/color/image_raw".format(cam), Image, self.rgb_callback)

        # subscribe to detections
        # self.detectnet_sub = rospy.Subscriber(
        #     "/detectnet/detections", Detection2DArray, self.detectnet_callback)

        # obstacle publisher
        self.obstacle_list_pub = rospy.Publisher('wizzybug/perception/obstacle_list', wizzybug_msgs.obstacleArray, queue_size=10)

        # initialize
        self.rgb, self.depth, self.detections = None, None, None

        # convert ros message to opencv
        self.bridge = CvBridge()


    def depth_callback(self, data):
    
        # save depth image
        self.depth = self.bridge.imgmsg_to_cv2(data)
        # cv2.imshow("", self.depth)
        # cv2.waitKey(1)

        # # obstacles in depth image
        obstacle_list, obstacle_mask = segmentation.segment_depth(self.depth)


        # publish obstacle_list
        self.publish_obstacles(obstacle_list)

    def publish_obstacles(self, obstacle_list):
        print("Detected {} obstacles".format(len(obstacle_list)))
        msg_object_array = wizzybug_msgs.obstacleArray()
        msg_object_array.header.stamp = rospy.Time.now()
        for obstacle in obstacle_list:
            msg_object = wizzybug_msgs.obstacle()
            msg_object.x.data = obstacle['x']
            msg_object.y.data = obstacle['y']
            msg_object.z.data = obstacle['z']
            msg_object.width.data = obstacle['width']
            msg_object.height.data = obstacle['height']
            msg_object.length.data = obstacle['length']  # should change to length
            msg_object.classification.data = obstacle['classification']
            msg_object_array.data.append(msg_object)
        self.obstacle_list_pub.publish(msg_object_array)
    
    def rgb_callback(self, data):
    
        # save depth image
        self.rgb = self.bridge.imgmsg_to_cv2(data)


    def detectnet_callback(self, data):

        #   get detections
        self.detections = data.detections

        # image for display
        D = (self.depth.astype(np.float)/255.0).astype(np.uint8)
        D = cv2.cvtColor(5*D, cv2.COLOR_GRAY2BGR)

        #   iterate over detections
        for detection in self.detections:

            # rectangle top-left
            start_point = (int(max(0, detection.bbox.center.x - detection.bbox.size_x/2.0)), 
                int(max(0, detection.bbox.center.y - detection.bbox.size_y/2.0)))
  
            # bottom right corner of rectangle 
            end_point = (int(min(self.depth.shape[1], detection.bbox.center.x + detection.bbox.size_x/2.0)),
                        int(min(self.depth.shape[0], detection.bbox.center.y + detection.bbox.size_y/2.0)))
                            
            # Blue color in BGR 
            color = (0, 255, 0) 
            
            # Line thickness of 2 px 
            thickness = 2
            
            # Using cv2.rectangle() method 
            # Draw a rectangle with blue line borders of thickness of 2 px 
            cv2.rectangle(D, start_point, end_point, color, thickness)
            cv2.rectangle(self.rgb, start_point, end_point, color, thickness)


        # cv2.imshow("depth", D)
        # cv2.imshow("rgb", self.rgb)
        # cv2.waitKey(1)

        # # use segnet output
        # segmentation.label_detections(
        #     obstacle_list, obstacle_mask, self.segnet)


if __name__ == '__main__':
    rospy.init_node('obstacle_detector', anonymous=True)
    i = ImageContainer()
    print("Obstacle Detector Initialized. waiting for image feed...")
    rospy.spin()
