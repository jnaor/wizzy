#!/usr/bin/env python

import rospy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3
from wizzybug_msgs.msg import lidar_data, obstacle, obstacleArray

# for how long to display markers
MARKER_DISPLAY_DURATION = 0.1


class Visualizer(object):

    def __init__(self, marker_display_duration):
        
        # initialize node
        rospy.init_node('sensing_visualization', log_level=rospy.DEBUG)

        # subscribe to lidar
        lidar_subscriber = rospy.Subscriber('/wizzy/lidar_proc', lidar_data, self.lidar_callback)
        
        # subscribe to camera results
        camera_subscriber = rospy.Subscriber('/wizzy/obstacle_list', obstacleArray, self.camera_callback)

        # markers publisher
        self.obstacles_publisher = rospy.Publisher('obstacle_marker', MarkerArray, queue_size = 10)

    def lidar_callback(self, lidar_data):
        pass

    def camera_callback(self, obstacle_array):

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

        # publish markers for detected 
        self.obstacles_publisher.publish(marker_array)

    def spin(self):
        rospy.spin()

if __name__ == "__main__":

    # initialize visualizer and run
    Visualizer(MARKER_DISPLAY_DURATION).spin()

