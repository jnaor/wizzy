#!/usr/bin/env python

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3
from wizzybug_msgs.msg import lidar_data, obstacle, obstacleArray

MARKER_DISPLAY_DURATION = 0.1

class Visualizer(object):

    def __init__(self, marker_display_duration):
        
        # initialize display duration for markers
        self.display_duration = rospy.duration(MARKER_DISPLAY_DURATION)

        # initialize node
        rospy.init_node('sensing_visualization')

        # subscribe to lidar
        lidar_subscriber = rospy.Subscriber('/wizzy/lidar_proc', lidar_data, handler.activation_callback)
        
        # subscrive to camera results
        camera_subscriber = rospy.Subscriber('/wizzy/obstacle_list', obstacleArray, handler.activation_callback)

        # markers publisher
        self.obstacles_publisher = rospy.Publisher('obstacle_markers', MarkerArray, queue_size = 10)

        # to hold obstacle markers array
        self.obstacle_markers = []


    def lidar_callback(self, lidar_data):
        pass

    def camera_callback(self, obstacleArray):
        pass

    def run():
        while not rospy.is_shutdown():
            pass


if __name__ == "__main__":

    # initialize visualizer and run
    Visualizer(MARKER_DISPLAY_DURATION).run()

