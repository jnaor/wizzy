#!/usr/bin/env python

import numpy as np
import cv2

import rospy

from segmentation import segment_depth
from cv_bridge import CvBridge, CvBridgeError

# inputs are images
from sensor_msgs.msg import Image

# output
from wizzybug_msgs.msg import obstacle, obstacleArray

from scipy.stats import mode

colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 0, 255),
          (128, 0, 0), (0, 128, 0), (0, 0, 128),
          (128, 0, 128),
          (128, 128, 0), (0, 128, 128)]

labels = {0: 'other', 1: 'wall', 2: 'floor', 3: 'cabinet/shelves',
          4: 'bed/pillow', 5: 'chair', 6: 'sofa', 7: 'table',
          8: 'door', 9: 'window', 10: 'picture/tv', 11: 'blinds/curtain',
          12: 'clothes', 13: 'ceiling', 14: 'books', 15: 'fridge',
          16: 'person', 17: 'toilet', 18: 'sink', 19: 'lamp',
          20: 'bathtub'}


class ObstacleDetector(object):
    """ Base class for vision-based object detection """

    # class-wide cv bridge (initialize on first use)
    bridge = None

    def __init__(self, camera):

        # initialize ros2opencv converter if not done already
        if ObstacleDetector.bridge is None:
            ObstacleDetector.bridge = CvBridge()

        # subscribe to depth
        rospy.Subscriber("/{}/depth/image_mono16".format(camera['name']), Image, self.process_depth)

        # subscribe to segmented rgb image topic
        rospy.Subscriber("/{}/segnet/class_mask".format(camera['name']), Image, self.segnet_callback)

        # initialize images
        self.segmentation_image, self.depth_image = None, None

        # initialize obstacle reporting structures
        self.obstacle_list, self.obstacle_mask = None, None

        # obstacle publisher
        self.obstacle_list_pub = rospy.Publisher('/wizzy/obstacle_list', obstacleArray, queue_size=10)

    def process_depth(self, msg):
        # get an opencv image from ROS
        self.depth_image = ObstacleDetector.bridge.imgmsg_to_cv2(msg, "passthrough")

        # detect obstacles based on depth
        self.obstacle_list, self.obstacle_mask = segment_depth(self.depth_image)

        # for ind, mask in enumerate(self.obstacle_mask):
        #     cv2.imshow(str(ind), 255*mask.astype(np.uint8))
        #
        # cv2.waitKey(1)
        # self.viewer.display(self.depth_image, self.obstacle_list, self.obstacle_mask)

        # publish obstacles TODO: should this be here?
        self.publish_obstacles()

    def publish_obstacles(self):
        msg_object_array = obstacleArray()
        msg_object_array.header.stamp = rospy.Time.now()
        for current_obstacle in self.obstacle_list:
            msg_object = obstacle()
            msg_object.x.data = current_obstacle['x']
            msg_object.y.data = current_obstacle['y']
            msg_object.z.data = current_obstacle['z']
            msg_object.width.data = current_obstacle['width']
            msg_object.height.data = current_obstacle['height']
            msg_object.length.data = current_obstacle['length']  # should change to length
            msg_object.classification.data = current_obstacle['classification']
            msg_object_array.data.append(msg_object)
        self.obstacle_list_pub.publish(msg_object_array)

    def segnet_callback(self, data):

        self.segmentation_image = ObstacleDetector.bridge.imgmsg_to_cv2(data)

        # use segnet output
        self.label_detections()

    def label_detections(self):
        # TODO: a more elegant way than a global variable
        global labels

        # for each detection
        for obstacle, mask in zip(self.obstacle_list, self.obstacle_mask):
            # mask segmentation image
            segmented_mask = cv2.bitwise_and(self.segmentation_image, mask)

            # find most common element
            most_common_class = mode(segmented_mask.flatten())

            # this will be the classification of the obstacle
            obstacle['class'] = labels[most_common_class]
            

if __name__ == '__main__':
    import json
    import os

    # initialize ROS node
    rospy.init_node('wizzybug_vision', log_level=rospy.DEBUG)

    # get camera configuration filename from ros
    config_filename = rospy.get_param('camera_config')

    print('found config filename {}'.format(config_filename))

    # read cameras from json
    try:
        with open(config_filename) as f:
            cameras = json.load(f)
    except IOError:
        rospy.logerr('cannot open camera configuration file {}. current directory is {}'.format(
            config_filename, os.getcwd()))

    # start detector per camera
    detectors = [ObstacleDetector(camera) for camera in cameras]
    rospy.spin()
