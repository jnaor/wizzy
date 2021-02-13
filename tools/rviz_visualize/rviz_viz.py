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
        self. marker = Marker(
                                type=Marker.ARROW,
                                id = 123,  # Make sure the ID is different for every marker you make!
                                pose=Pose(Point(0, -0.1, 0), Quaternion(0, 0, 0, 1)),
                                scale=Vector3(1, 1, 1),
                                header=Header(frame_id='base_link'),
                                color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)                                

    def lidar_dist_to_obstacle_callback(self, data):
        self.lidar_dist = data.visible_floor_distance
        rospy.loginfo("self.lidar_dist : " + str(self.lidar_dist))

        self.marker.pose = Pose(Point(self.lidar_dist, -0.1, 0), Quaternion(0, 0, 0, 1))
        self.marker.header.stamp = rospy.Time.now()
        self.marker_publisher.publish(self.marker)


def my_main(inputs_container):
    # Initialize and run node	
    rospy.init_node('rviz_viz_node')

    lidar_dist_to_obstacle_subscriber = rospy.Subscriber('/wizzy/lidar_proc',
                                                        lidar_data,
                                                        inputs_container.lidar_dist_to_obstacle_callback,
                                                        queue_size=1)


    rospy.spin()

if __name__ == '__main__':
    inputs_container = CallbackItems()

    try:
        my_main(inputs_container)
    except rospy.ROSInterruptException:
        rospy.logerr(rospy.get_name() + ': Got ROS Interrupt Exception')
    
