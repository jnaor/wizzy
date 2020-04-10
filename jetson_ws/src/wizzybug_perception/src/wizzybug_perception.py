import os
import logging
import cv2

from ros_interface import RosInterface
from segmentation import segment_depth
from realsense import Realsense

def main():
     # initialize logging
     logging.basicConfig(format='Perception Module %(asctime)s %(message)s', level=logging.DEBUG)
    
     # initialize ROS
     logging.info('Initializing ROS')
     ros_interface = RosInterface()
     
     # initialize realsense grabber
     logging.info('Initialize Realsense Cameras')
     realsense = Realsense()

     # start
     logging.info('Start operation')

     while not ros_interface.is_shutdown():
         # grab images
         depth_image, rgb_image, colorized_depth_image = realsense.grab()

         # detect obstacles
         obstacle_list, obstacle_mask = segment_depth(depth_image)

         # draw images and obstacles
         # cv2.imshow("depth map", colorized_depth_image)
         # cv2.imshow("camera image with obstacles", cv2.cvtColor(image_with_obstacles, cv2.COLOR_RGB2BGR))
         # cv2.waitKey(1)

         # publish in ROS
         if len(obstacle_list):
             ros_interface.publish_obstacles(obstacle_list)
         # ros_interface.publish_image(rgb_image, "image")

def stand_alone_test():
    ros_interface = RosInterface()

    while not ros_interface.is_shutdown():
        obstacle_list = list()
        obstacle_list.append({'classification': 'Person', 'x': 1., 'y': 0., 'z': 0., 'width': 3., 'height': 4., 'depth': 0.})
        ros_interface.publish_obstacles(obstacle_list)

if __name__ == '__main__':
    main()

    # stand_alone_test()
