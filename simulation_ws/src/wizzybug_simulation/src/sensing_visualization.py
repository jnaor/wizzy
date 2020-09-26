#!/usr/bin/env python

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3
from wizzybug_msgs.msg import lidar_data, obstacle, obstacleArray

# visualization frequency detemined by this
SLEEP_DURATION = 0.05

# for how long to display markers
MARKER_DISPLAY_DURATION = 0.1


class Visualizer(object):

    def __init__(self, marker_display_duration):
        
        # initialize node
        rospy.init_node('sensing_visualization', log_level=rospy.DEBUG)

        # initialize display duration for markers
        self.display_duration = rospy.duration(MARKER_DISPLAY_DURATION)

        # subscribe to lidar
        lidar_subscriber = rospy.Subscriber('/wizzy/lidar_proc', lidar_data, self.lidar_callback)
        
        # subscribe to camera results
        camera_subscriber = rospy.Subscriber('/wizzy/obstacle_list', obstacleArray, self.camera_callback)

        # markers publisher
        self.obstacles_publisher = rospy.Publisher('obstacle_marker', MarkerArray, queue_size = 10)

    def lidar_callback(self, lidar_data):
        rospy.logdebug('in lidar callback')
        pass

    def camera_callback(self, obstacle_array):
        rospy.logdebug('in camera callback')

        # to hold obstacle markers array
        markerArray = MarkerArray()

        # go over received array and add markers per obstacle
        for obstacle in obstacle_array:

            # new marker for this obstacle
            marker = Marker()

            # marker attributes
            marker.header.frame_id = "/obstacle_marker"

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

            markerArray.markers.append(marker)

        # publish markers for detected 
        self.obstacles_publisher.publish(markerArray)

    def spin(self):
        rospy.spin()

if __name__ == "__main__":

    # initialize visualizer and run
    Visualizer(MARKER_DISPLAY_DURATION).spin()

