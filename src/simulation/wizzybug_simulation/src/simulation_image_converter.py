#!/usr/bin/env python

import sys
import rospy
import json
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImageConverter:

    def __init__(self, cam_name):
        # initialize ros2opencv bridge
        self.bridge = CvBridge()

        # subscribe to image from simulator
        self.image_sub = rospy.Subscriber('/{}/depth/image_rect_raw'.format(cam_name), Image, self.callback)

        # publish result
        self.image_pub = rospy.Publisher("/{}/depth/image_mono16".format(cam_name), Image, queue_size=100)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")

            # convert to millimeters and 16-bit like realsense output
            cv_image = (1000 * cv_image).astype(np.uint16)

        except CvBridgeError as e:
            rospy.logerr(e)

        try:
            # publish as 16-bit mono
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "mono16"))
        except CvBridgeError as e:
            rospy.logerr(e)


def main(args):
    rospy.init_node('simulation_image_converter', anonymous=True)

    # read camera configuration. default to local if no parameter set by ros
    config_file = rospy.get_param('vision_config')
    rospy.loginfo('opening configuration file {}'.format(config_file))

    with open(config_file) as f:
        config = json.load(f)

    [ImageConverter(cam_name=cam['name']) for cam in config['cameras']]

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Simulation image converter shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
