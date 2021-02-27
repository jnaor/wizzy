#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from wizzybug_msgs.msg import obstacle, obstacleArray

import time
import math


class ObstacleHandler:
    def __init__(self):
        self.markers_publisher = rospy.Publisher('obstacles', MarkerArray, queue_size=100)
        self.obstacle_subscriber = rospy.Subscriber("/wizzy/obstacle_list", obstacleArray, self.visualize_obstacles)
        self.marker_array = MarkerArray()

    def visualize_obstacles(self, obstacle_array):

        # clear marker list for a fresh new start
        self.marker_array.markers = []

        # go over received array and add markers per obstacle
        for obstacle_id, obstacle_data in enumerate(obstacle_array.data):

            # new marker for this obstacle
            marker = Marker()

            # marker attributes
            marker.header.frame_id = "base_footprint"
            marker.header.stamp = rospy.Time.now()

            marker.type = marker.SPHERE
            marker.scale = Vector3(0.5, 0.5, 0.5)
            marker.color = ColorRGBA(1.0, 1.0, 0.0, 1.0)
            marker.pose.orientation = Quaternion(0, 0, 0, 1)

            # get location from message
            marker.pose.position.x = obstacle_data.x  # + 0.5
            marker.pose.position.y = obstacle_data.y
            marker.pose.position.z = obstacle_data.z

            # id for this marker
            # added arbitrary offset so that it won't collide with other visualization nodes
            marker.id = obstacle_id + 135

            # finally, add to list
            self.marker_array.markers.append(marker)

        self.markers_publisher.publish(self.marker_array)

def main():
    # Initialize and run node
    rospy.init_node('obstacle_visualizer')
    obstacle_handler = ObstacleHandler()
    rospy.spin()

    # while not rospy.is_shutdown():
    #     pass


if __name__ == '__main__':
    main()
