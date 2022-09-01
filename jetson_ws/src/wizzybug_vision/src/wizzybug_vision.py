#!/usr/bin/env python

import time
import numpy as np
import cv2

from itertools import chain

import rospy

from segmentation import segment_depth
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, PointCloud2
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

        return self.obstacle_list, self.obstacle_mask

        if self.camera.depth_image is None:
            rospy.logwarn('no image received for camera {}'.format(self.camera.name))
            return

        # restrict to places where we have finite readings
        finite_indices = np.isfinite(x) & np.isfinite(z)
        x, z = x[finite_indices], z[finite_indices]

        # sort by x
        sorted_indices = np.argsort(x)
        x, z = x[sorted_indices], z[sorted_indices]

        # check that both x and z are not empty
        if len(x) * len(z) == 0:
            rospy.logerr("invalid laser scan readings")
            return

        # locations assumed to be the ground
        ground_indices = np.where((x > LidarProcess.GROUND_PLANE_ESTIMATION_MIN_DISTANCE) &
                               (x < LidarProcess.GROUND_PLANE_ESTIMATION_MAX_DISTANCE))
        ground_x, ground_z = x[ground_indices], z[ground_indices]

        if min(len(ground_x), len(ground_z)) == 0:
            rospy.logwarn('unable to detect ground readings')
            return

        # estimate ground line using RANSAC
        try:
            ransac = RANSACRegressor(random_state=0).fit(ground_x.reshape(-1, 1), ground_z)

        except ValueError as e:
            rospy.logwarn('RANSAC error {}'.format(e))
            return

        # estimate line at ground_x
        l = ransac.predict(ground_x.reshape(-1, 1))

        # keep ransac score
        ransac_score = np.abs(ransac.score(ground_x.reshape(-1, 1), ground_z))

        # if readings do not fit a line then report error
        if ransac_score > LidarProcess.GROUND_PLANE_ESTIMATION_THRESHOLD:
            rospy.logwarn('unable to detect ground. score is {}'.format(ransac_score))
            return

        # ransac inliers
        inlier_mask = ransac.inlier_mask_

        # ransac outliers
        outlier_mask = np.logical_not(inlier_mask)

        # find index of last outlier in original array
        try:
            last_outlier_index = ground_indices[0][np.max(np.nonzero(outlier_mask)[0])]

        # if there are no outliers
        except ValueError:
            # take last ground point
            last_outlier_index = 0

        # record lidar height (minus sign because the lidar is above the floor)
        ld.lidar_height = -ransac.predict([[0]])[0]

        # floor angle
        inclination = np.arctan((l[0] - l[-1]) / (x[-1] - x[0]))
        ld.floor_inclination_degrees = np.rad2deg(inclination)

        # rotation matrix to make ground level
        R = np.array([[np.cos(inclination), -np.sin(inclination)], [np.sin(inclination), np.cos(inclination)]])

        # transform readings
        T = np.matmul(R, np.vstack((x, z)))

        # subtract ground height
        T[1, :] += ld.lidar_height

        # difference between successive z measurements
        diff_x, diff_z = np.abs(np.diff(T[0])), np.abs(np.diff(T[1]))

        # skip to after the last outlier
        diff_z[:last_outlier_index+1] = 0

        # x gaps
        x_gap = binary_erosion(diff_x < LidarProcess.MAX_FLOOR_X_DIFF)

        # detect gap in z
        try:
            z_gap = min(np.where(np.bitwise_and(x_gap, diff_z > LidarProcess.MAX_FLOOR_Z_DIFF))[0])
            ld.visible_floor_distance = x[z_gap - 1]
        except ValueError:
            ld.visible_floor_distance = np.max(x)
            # rospy.logwarn('no z gap detected; max range visibility for now ({})'.format(msg.range_max))


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
        msg_object.x.data = current_obstacle['x']
        msg_object.y.data = current_obstacle['y']
        msg_object.z.data = current_obstacle['z']
        msg_object.width.data = current_obstacle['width']
        msg_object.height.data = current_obstacle['height']
        msg_object.length.data = current_obstacle['length']  # should change to length
        msg_object.classification.data = current_obstacle['classification']
        msg_object_array.data.append(msg_object)
    obstacle_publisher.publish(msg_object_array)


class Camera(object):
    def __init__(self, config):
        # save parameters
        self.name, self.serial = config['name'], config['serial']

        # placeholders for images and pointcloud
        self.color_image, self.depth_image, self.pointcloud = None, None, None

        # listen for camera transformation
        listener = tf.TransformListener()

        try:
            # wait till transform is available
            listener.waitForTransform('/base_link', '/{}_link'.format(self.name), rospy.Time(), rospy.Duration(10.0))

            (translation, rotation_quat) = listener.lookupTransform(
                '/base_link', '/{}_link'.format(self.name), rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('cannot read position of {} on bug'.format(self.name))

        # get rotation part of pose
        self.pose = quaternion_matrix(rotation_quat)

        # add translation
        self.pose[:3, 3] = translation

    def grab(self):
        pass


class ROSCamera(Camera):
    def __init__(self, config):
        # generic camera
        super(ROSCamera, self).__init__(config)

        # subscribe to depth
        rospy.Subscriber("/{}/depth/image_mono16".format(self.name), Image, self.save_depth)

        # subscribe to pointcloud TODO: did not "compile"
        # rospy.Subscriber("/{}/depth/points".format(self.name), PointCloud2, self.save_pointcloud)


        # initialize ros2opencv converter
        self.cv_bridge = CvBridge()

    def save_depth(self, msg):
        """ callback activated when ros depth image message received """
        # create an opencv image from ROS
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, "passthrough")

    def save_pointcloud(self, msg):

        # save pointcloud
        self.pointcloud = msg.points



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
        if 'depth' in image.keys(): self.depth_image = image['depth'], self.pointcloud = image["pointcloud"]
        if 'color' in image.keys(): self.color_image = image['color']


def visualize_obstacles(self, obstacle_array):

    # to hold obstacle markers array
    marker_array = MarkerArray()

    # go over received array and add markers per obstacle
    for obstacle_id, obstacle in enumerate(obstacle_array.data):

        # new marker for this obstacle
        marker = Marker()

        # marker attributes
        marker.header.frame_id = "base_footprint"

        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        
        # get location from message
        marker.pose.position.x = obstacle.x.data
        marker.pose.position.y = obstacle.y.data
        marker.pose.position.z = obstacle.z.data

        # how long to show
        marker.lifetime = rospy.Duration(MARKER_DISPLAY_DURATION)

        # id for this marker
        marker.id = obstacle_id

        # finally, add to list
        marker_array.markers.append(marker)
    
    # TODO: ugly global
    visualization_publisher.publish(marker_array)


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

    # visualization on?
    visualization = rospy.get_param('visualization', False)

    if visualization:

        from visualization_msgs.msg import Marker, MarkerArray

        # visualization publisher if requested
        visualization_publisher = rospy.Publisher('obstacle_marker', MarkerArray, queue_size = 10)

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

        if visualization:
            # publish markers
            visualize_obstacles(obstacle_list)

        # go to sleep till next request
        rate.sleep()


