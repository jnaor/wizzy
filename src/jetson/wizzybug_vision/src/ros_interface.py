#!/usr/bin/env python
import sys
import os

import numpy as np
import cv2
from ros import *
import std_msgs.msg
# from vision_msgs.msg import BoundingBox3D
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge


sys.path.append(os.path.join(os.getcwd(), 'devel/lib/python2.7/dist-packages'))
sys.path.append(os.path.join(os.getcwd(), 'devel/lib/python3/dist-packages'))

import wizzybug_msgs.msg


class RosInterface():
    def __init__(self):
        self.obstacles_list_publisher = rospy.Publisher('wizzy/obstacle_list', wizzybug_msgs.msg.obstacleArray, queue_size=10)
        # self.obstacles_list_publisher = rospy.Publisher('wizzy/rviz/obstacle_list', wizzybug_msgs.msg.obstacleArray, queue_size=10)
        self.image_publisher = rospy.Publisher('wizzy/realsense2/image', CompressedImage, queue_size=10)
        self.depth_image_publisher = rospy.Publisher('realsense/depth_image', CompressedImage, queue_size=10)
        self.annotated_image_publisher = rospy.Publisher('realsense/annotated_image', CompressedImage, queue_size=10)
        rospy.init_node('wizzybug_vision', anonymous=True)
        pass

    def is_shutdown(self):
        return rospy.is_shutdown()

    def publish(self):
        pass

    def publish_obstacles(self, obstacle_list):
        print(obstacle_list)
        msg_object_array = wizzybug_msgs.msg.obstacleArray()
        msg_object_array.header.stamp = rospy.Time.now()
        for obstacle in obstacle_list:
            msg_object = wizzybug_msgs.msg.obstacle()
            msg_object.x.data = obstacle['x']
            msg_object.y.data = obstacle['y']
            msg_object.z.data = obstacle['z']
            msg_object.width.data = obstacle['width']
            msg_object.height.data = obstacle['height']
            msg_object.depth.data = obstacle['depth']
            msg_object.classification.data = obstacle['classification']
            msg_object_array.data.append(msg_object)
        self.obstacles_list_publisher.publish(msg_object_array)





    def publish_image(self, image, type):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', cv2.cvtColor(image, cv2.COLOR_RGB2BGR))[1]).tostring()
        # msg = Image()
        # msg.header.stamp = rospy.Time.now()
        # msg.data = image

        if(type == "image"):
            self.image_publisher.publish(msg)
        elif(type == "depth_image"):
            self.depth_image_publisher.publish(msg)
        elif (type == "annotated_image"):
            self.annotated_image_publisher.publish(msg)
