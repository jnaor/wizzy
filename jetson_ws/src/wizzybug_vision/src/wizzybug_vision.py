#!/usr/bin/env python

import numpy as np
import cv2

from itertools import chain

import rospy
import tf
from tf.transformations import quaternion_matrix
import tf2_py

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

    def __init__(self, camera, base_cam):

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

        # listen for camera pose
        listener = tf.TransformListener()

        # save transformation from base camera
        self.translation, rotation_quaternion = listener.lookupTransform('/{}'.format(base_cam), '/{}'.format(camera),
                                                                   rospy.Time(0))
        # convert to matrix
        self.rotation = quaternion_matrix(rotation_quaternion)

    def process_depth(self, msg):
        # get an opencv image from ROS
        self.depth_image = ObstacleDetector.bridge.imgmsg_to_cv2(msg, "passthrough")

        # detect obstacles based on depth
        self.obstacle_list, self.obstacle_mask = segment_depth(self.depth_image, self.translation, self.rotation)

        # for ind, mask in enumerate(self.obstacle_mask):
        #     cv2.imshow(str(ind), 255*mask.astype(np.uint8))
        #
        # cv2.waitKey(1)
        # self.viewer.display(self.depth_image, self.obstacle_list, self.obstacle_mask)

        # publish obstacles TODO: should this be here?
        self.publish_obstacles()

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
            
def publish_obstacles(obstacle_publisher, obstacle_list):
    msg_object_array = obstacleArray()
    msg_object_array.header.stamp = rospy.Time.now()
    for current_obstacle in obstacle_list:
        msg_object = obstacle()
        msg_object.x.data = current_obstacle['x']
        msg_object.y.data = current_obstacle['y']
        msg_object.z.data = current_obstacle['z']
        msg_object.width.data = current_obstacle['width']
        msg_object.height.data = current_obstacle['height']
        msg_object.length.data = current_obstacle['length']  # should change to length
        msg_object.classification.data = current_obstacle['classification']
        msg_object_array.data.append(msg_object)
    obstacle_publisher.publish(msg_object_array)


if __name__ == '__main__':
    import json
    import os

    # initialize ROS node
    rospy.init_node('wizzybug_vision', log_level=rospy.DEBUG)

    # to hold list of camera names
    cameras = list()

    # get published topics
    published_topics = dict(rospy.get_published_topics())

    # search for topics containing camera string
    for topic_name in published_topics.keys():

        rospy.logdebug('adding obstacle detector for {}'.format(topic_name))

        # if this is a camera
        if "_camera" in topic_name:
            cameras.append(topic_name)

    # run at 10 hertz TODO: parameter
    rate = rospy.Rate(10)

    # obstacle publisher
    obstacle_list_pub = rospy.Publisher('/wizzy/obstacle_list', obstacleArray, queue_size=10)

    # start detector per camera
    detectors = [ObstacleDetector(camera, base_cam=cameras[0]) for camera in cameras]

    while not rospy.is_shutdown():

        # concatenate obstacles from all cameras
        combined = [detector.obstacle_list for detector in detectors]
        obstacle_list = [obstacle for obstacle in chain.from_iterable(combined)]

        # publish
        publish_obstacles(obstacle_list_pub, obstacle_list)

        # go to sleep till next request
        rate.sleep()
