from __future__ import print_function

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self, cam="camera"):
    self.image_pub = rospy.Publisher("/{}/depth/image_rect_raw".format(cam),Image,queue_size=100)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber('/front_camera/depth/image_rect_raw',Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")

      # convert to millimeters and 16-bit like realsense output
      cv_image = (1000*cv_image).astype(np.uint16)

      print(cv_image)

    except CvBridgeError as e:
      print(e)

    try:
      # publish as 16-bit mono
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "mono16"))
    except CvBridgeError as e:
      print(e)

def main(args):
  
  rospy.init_node('simulation_image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)