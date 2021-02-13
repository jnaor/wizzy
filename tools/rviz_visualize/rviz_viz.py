#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

from wizzybug_msgs.msg import ttc, lidar_data

import math
import time

# To solve frame TF issue, need to run :
# $ rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map base_link 10

class CallbackItems:
    def __init__(self):
        self.lidar_dist = 100.0
    def lidar_dist_to_obstacle_callback(self, data):
        self.lidar_dist = data.visible_floor_distance
        print ("self.lidar_dist : " + str(self.lidar_dist))


def my_main(inputs_container):
    """
    One place to initalize and run the code
    """
    # Initialize and run node	
    rospy.init_node('my_node')

    marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)

    lidar_dist_to_obstacle_subscriber = rospy.Subscriber('/wizzy/lidar_proc',
                                                        lidar_data,
                                                        inputs_container.lidar_dist_to_obstacle_callback,
                                                        queue_size=1)

    marker = Marker(
    #                type=Marker.TEXT_VIEW_FACING,
        type=Marker.ARROW,
        id = 123,  # Make sure the ID is different for every marker you make!
        #   action=Marker.ADD,
        #   lifetime=rospy.Duration(1.5),
        pose=Pose(Point(0, -0.1, 0), Quaternion(0, 0, 0, 1)),
        scale=Vector3(1, 1, 1),
        header=Header(frame_id='base_link', stamp = rospy.Time.now()),
        color=ColorRGBA(0.0, 1.0, 0.0, 0.8))

# lidar_data.visible_floor_distance

    while not rospy.is_shutdown():
        marker.scale.x = 2 + math.sin(time.time())
        marker.header.stamp = rospy.Time.now()
        marker_publisher.publish(marker)
        rospy.sleep(0.5)

def idan_main():
    # Initialize and run node
    rospy.init_node('my_node')
    marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
    header = Header()
    header.frame_id = 'base_link'  # This could be 'rplidar_link' but then the marker Z pos would need to change.
    header.stamp = rospy.Time.now()

    marker = Marker()
    marker.type = Marker.ARROW
    marker.id = 123  # Make sure the ID is different for every marker you make!
    marker.pose.position = Point(0, -0.1, 0)
    marker.scale = Vector3(1, 0.1, 0.1)
    marker.header = header
    marker.color=ColorRGBA(0.0, 1.0, 0.0, 0.8)

    while not rospy.is_shutdown():
        marker.scale.x = 2 + math.sin(time.time())
        marker.header.stamp = rospy.Time.now()
        marker_publisher.publish(marker)
        rospy.sleep(0.5)

if __name__ == '__main__':
    inputs_container = CallbackItems()

    my_main(inputs_container)
