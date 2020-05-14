#!/usr/bin/env python
from __future__ import print_function

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImageConverter:
    """ 'convert' the RealSense RGB format to the BGR format jetson-inference likes """

    def __init__(self, cam="camera"):
        self.image_pub = rospy.Publisher("image_topic_bgr8_{}".format(cam), Image, queue_size=100)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/{}/color/image_raw".format(cam), Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


def main(args):
    # initialize ros node
    rospy.init_node('image_converter', anonymous=True)

    # get published topics
    published_topics = dict(rospy.get_published_topics())

    # to hold list of converters
    converters = list()

    # search for topics containing camera string
    for topic_name in published_topics.keys():

        # if this is a camera
        if "_camera" in topic_name:
            rospy.logdebug('adding image converter for {}'.format(topic_name))

            converters.append(ImageConverter(cam=topic_name))
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
