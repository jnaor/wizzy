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

    def __init__(self, camera_names):

        # initialize ros2opencv converter
        self.bridge = CvBridge()

        # save camera names
        self.camera_names = camera_names

        # subscribe to camera readings
        for camera_name in camera_names:
            # subscribe to depth
            rospy.Subscriber("/{}/depth/image_mono16".format(camera_name), Image, self.process_depth)

            # subscribe to segmented rgb image topic
            rospy.Subscriber("/{}/segnet/class_mask".format(camera_name), Image, self.segnet_callback)

        # initialize publisher
        self.publisher = rospy.Publisher('wizzy/obstacle_list', obstacleArray, queue_size=10)

        # initialize images
        self.segmentation_image, self.depth_image = None, None

        # initialize obstacle reporting structures
        self.obstacle_list, self.obstacle_mask = None, None

        # obstacle publisher
        self.obstacle_list_pub = rospy.Publisher('/wizzy/vision/obstacle_list', obstacleArray, queue_size=10)

        # TODO: remove
        self.viewer = Viewer(5)

    def process_depth(self, msg):
        # get an opencv image from ROS
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")

        # detect obstacles based on depth
        self.obstacle_list, self.obstacle_mask = segment_depth(self.depth_image)

    def publish_obstacles(self):
        print("Detected {} obstacles".format(len(self.obstacle_list)))
        msg_object_array = obstacleArray()
        msg_object_array.header.stamp = rospy.Time.now()
        for obstacle in self.obstacle_list:
            msg_object = obstacle()
            msg_object.x.data = obstacle['x']
            msg_object.y.data = obstacle['y']
            msg_object.z.data = obstacle['z']
            msg_object.width.data = obstacle['width']
            msg_object.height.data = obstacle['height']
            msg_object.length.data = obstacle['length']  # should change to length
            msg_object.classification.data = obstacle['classification']
            msg_object_array.data.append(msg_object)
        self.obstacle_list_pub.publish(msg_object_array)

    def segnet_callback(self, data):

        self.segmentation_image = self.bridge.imgmsg_to_cv2(data)

        # use segnet output
        self.label_detections()

        # publish obstacles TODO: should this be here?
        self.publish_obstacles()

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


class Viewer(object):
    def __init__(self, max_objects):

        # max number for display
        self.max_objects = max_objects

        # initialize figure
        self.fig = None

        # initialize handles
        self.handles = dict()

    def display(self, vis_image, obstacle_list, obstacle_mask):
        from matplotlib import pylab as plt

        # create figure if it does not already exist
        if self.fig is None:
            self.fig, self.ax = plt.subplots(1, self.max_objects)

            # remove ticks
            for ind in range(self.max_objects):

                # get segment if it exists
                if ind < len(obstacle_mask):
                    self.handles[ind] = self.ax[ind].imshow(cv2.bitwise_and(vis_image, vis_image,
                                                                            mask=obstacle_mask[ind].astype(np.uint8)))
                else:
                    self.handles[ind] = self.ax[ind].imshow(np.zeros((480, 640), dtype=np.uint8))

                self.ax[ind].set_xticklabels([])
                self.ax[ind].set_yticklabels([])

            plt.tight_layout()

        else:
            for ind in range(self.max_objects):
                # get segment if it exists
                if ind < len(obstacle_mask):
                    self.handles[ind].set_data(cv2.bitwise_and(vis_image, vis_image,
                                                               mask=obstacle_mask[ind].astype(np.uint8)))

                    self.ax[ind].set_title('depth {}'.format(obstacle_list[ind]["x"]))
                else:
                    self.handles[ind].set_data(np.zeros((480, 640), dtype=np.uint8))
                    self.ax[ind].set_title('')

            self.fig.canvas.draw()
            plt.pause(0.01)


if __name__ == '__main__':
    rospy.init_node('wizzybug_vision', log_level=rospy.DEBUG)

    # TODO: read camera names from json or something
    obstacle_detector = ObstacleDetector(['front_camera'])
    rospy.spin()
