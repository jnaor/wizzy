
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

import segmentation

class ImageContainer:
  def __init__(self, cam="camera"):
    
    self.depth_sub = rospy.Subscriber("/{}/depth/image_rect_raw".format(cam), Image, self.depth_callback)
    self.segnet_sub = rospy.Subscriber("/segnet/class_mask", Image, self.segnet_callback)
    self.depth = None
    self.segnet = None

    self.bridge = CvBridge()

  def depth_callback(self, data):
    self.depth = data.data
    # print('here')
    self.depth = self.bridge.imgmsg_to_cv2(data)
    # cv2.imshow("depth", self.depth)
    # cv2.waitKey(1)

  def segnet_callback(self, data):
    
    self.segnet = self.bridge.imgmsg_to_cv2(data)
    # print("in segnet")
    # print(self.segnet)
    # cv2.imshow("segmentation", 10*self.segnet)
    # cv2.waitKey(1)

    # obstacles in depth image
    obstacle_list, obstacle_mask = segmentation.segment_depth(self.depth)

    # use segnet output 
    segmentation.label_detections(obstacle_list, obstacle_mask, self.segnet)


if __name__=='__main__':
    rospy.init_node('container', anonymous=True)
    i = ImageContainer()
    rospy.spin()
