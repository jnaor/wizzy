#!/usr/bin/env python

import time
import numpy as np
import cv2

from itertools import chain

import rospy

from segmentation import segment_depth
from cv_bridge import CvBridge, CvBridgeError

# inputs are images
from sensor_msgs.msg import Image
import tf
from tf.transformations import quaternion_matrix

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

    def __init__(self, camera):

        # use function to allow overloading of image grabber (e.g. from gazebo)
        self.camera = camera

        # initialize obstacle reporting structures
        self.obstacle_list, self.obstacle_mask = None, None

        rospy.loginfo('started camera {}'.format(camera.serial))

    def grab(self):
        self.camera.grab()

    def process_depth(self):

        # default to no detections
        self.obstacle_list, self.obstacle_mask = [], []

        if self.camera.depth_image is None:
            return

        # blacken out bottom of image to get rid of floor TODO: actual calculation
        num_black_rows = 150

        # need this since for some reason depth image is not writeable
        D = self.camera.depth_image.copy()

        D[-num_black_rows:, :] = 0

        # detect obstacles based on depth
        self.obstacle_list, self.obstacle_mask = segment_depth(D)

        # transform according to camera pose
        for detection in self.obstacle_list:
            # calculate with camera pose
            loc = np.matmul(self.camera.pose, [[detection['x']], [detection['y']], [detection['z']], [1]])
            dim = np.matmul(self.camera.pose, [[detection['width']], [detection['length']], [0], [1]])

            # update results
            detection['x'], detection['y'], detection['z'] = loc[0], loc[1], loc[2]
            detection['width'], detection['length'] = abs(dim[0]), abs(dim[1])

    def segnet_callback(self, data):

        self.segmentation_image = ObstacleDetector.cv_bridge.imgmsg_to_cv2(data)

        # use segnet output
        self.label_detections()

    def label_detections(self):
        # TODO: a more elegant way than a global variable
        global labels

        # for each detection
        for curr_obstacle, mask in zip(self.obstacle_list, self.obstacle_mask):
            # mask segmentation image
            segmented_mask = cv2.bitwise_and(self.segmentation_image, mask)

            # find most common element
            most_common_class = mode(segmented_mask.flatten())

            # this will be the classification of the obstacle
            curr_obstacle['class'] = labels[most_common_class]


class CameraFactory(object):
    @staticmethod
    def get_camera(camera_config):
        if camera_config['type'] == 'ros':
            return ROSCamera(camera_config)

        else:
            return DirectCamera(camera_config)


def publish_obstacles(obstacle_publisher, obstacle_list):
    msg_object_array = obstacleArray()
    msg_object_array.header.stamp = rospy.Time.now()
    for current_obstacle in obstacle_list:
        msg_object = obstacle()
        msg_object.x = current_obstacle['x']
        msg_object.y = current_obstacle['y']
        msg_object.z = current_obstacle['z']
        msg_object.width = current_obstacle['width']
        msg_object.height = current_obstacle['height']
        msg_object.length = current_obstacle['length']  # should change to length
        msg_object.classification = current_obstacle['classification']
        msg_object_array.data.append(msg_object)

    obstacle_publisher.publish(msg_object_array)


class Camera(object):
    def __init__(self, config):
        # save parameters
        self.name, self.serial = config['name'], config['serial']

        # placeholders for images
        self.color_image, self.depth_image = None, None

        # listen for camera transformation
        listener = tf.TransformListener()

        try:
            time.sleep(3)  # wait till transform is available
            child_transform = '/{}_link'.format(self.name)
            listener.waitForTransform('/base_link', child_transform, rospy.Time(), rospy.Duration(10))

            (translation, rotation_quat) = listener.lookupTransform(
                '/base_link', '/{}_link'.format(self.name), rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('cannot read position of {} on bug'.format(self.name))

        # get rotation part of pose
        self.pose = quaternion_matrix(rotation_quat)

        # add translation
        self.pose[:3, 3] = translation

        # pose matrix seems ok
        # print('#############################################################################')
        # print(self.pose)
        # print('#############################################################################')

    def grab(self):
        pass


class ROSCamera(Camera):
    def __init__(self, config):
        # generic camera
        super(ROSCamera, self).__init__(config)

        # subscribe to depth
        rospy.Subscriber("/{}/depth/image_mono16".format(self.name), Image, self.save_depth)
        # rospy.Subscriber("/front_cam/depth/image_rect_raw".format(self.name), Image, self.save_depth)


        # initialize ros2opencv converter
        self.cv_bridge = CvBridge()

    def save_depth(self, msg):
        """ callback activated when ros depth image message received """
        # create an opencv image from ROS
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, "passthrough")


class DirectCamera(Camera):
    def __init__(self, config):
        # generic camera
        super(DirectCamera, self).__init__(config)

        # grabber from directly connected camera
        from Grabber import Grabber

        # initialize
        self.grabber = Grabber(self.serial, self.name, config['width'],
                               config['height'], config['framerate'])

        # start
        self.grabber.start(depth=True)

    def grab(self):
        # get new image
        image = self.grabber.grab()

        # hold in internal structures
        if 'depth' in image.keys(): self.depth_image = image['depth']
        if 'color' in image.keys(): self.color_image = image['color']


if __name__ == '__main__':
    import json
    import os

    # initialize ROS node
    rospy.init_node('wizzybug_vision', log_level=rospy.DEBUG)

    # read camera configuration. default to local if no parameter set by ros
    config_file = rospy.get_param('vision_config', '../config/vision_config.json')
    rospy.loginfo('opening configuration file {}'.format(config_file))

    with open(config_file) as f:
        config = json.load(f)

    # obstacle publisher
    obstacle_list_pub = rospy.Publisher('/wizzy/obstacle_list', obstacleArray, queue_size=10)

    # initialize cameras
    cameras = list()

    for camera_config in config['cameras']:
        try:
            cameras.append(CameraFactory().get_camera(camera_config))
        except RuntimeError as runtime_error:
            rospy.logwarn(runtime_error)
            rospy.logwarn('cannot open {}. running without it'.format(camera_config['name']))

    # start detector per camera
    detectors = [ObstacleDetector(camera) for camera in cameras]

    # run at hz specified in config
    rate = rospy.Rate(config['rate'])


    # as long as ros is alive
    while not rospy.is_shutdown():

        # for each of the detectors
        for detector in detectors:

            # grab
            detector.grab()

            # and process
            detector.process_depth()

        # concatenate obstacles from all cameras
        combined = [detector.obstacle_list for detector in detectors]

        obstacle_list = [curr_obstacle for curr_obstacle in chain.from_iterable(combined)]

        # publish if not empty
        if len(obstacle_list) > 0:
            publish_obstacles(obstacle_list_pub, obstacle_list)
        # else:
        #     print('empty obstacle list')

        # go to sleep till next request
        rate.sleep()


