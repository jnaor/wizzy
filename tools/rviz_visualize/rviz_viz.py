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
        # Initialize display data
        self.lidar_obst_dist = 0.0
        self.ttc_time = 0.0
        # self.ttc_obst_dist = 0.0
        self.ttc_azimuth = 0.0

        # Initialize markers
        self.marker_obstacle = Marker(
                                type=Marker.LINE_STRIP,                                
                                id = 123,  # Make sure the ID is different for every marker
								pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
                                points=[Point(0, 0, 0), Point(1, 0, 0)],
                                scale=Vector3(1, 1, 1),
                                header=Header(frame_id='base_link'),
                                color=ColorRGBA(0.0, 1.0, 0.0, 0.8))

        self.marker_chair = Marker(
                                type=Marker.CUBE,
                                id = 124,
                                pose=Pose(Point(-0.5, 0, 0), Quaternion(0, 0, 0, 1)),
                                scale=Vector3(1, 1, 1),
                                header=Header(frame_id='base_link'),
                                color=ColorRGBA(0.0, 0.0, 1.0, 0.8))

        self.marker_dist_text = Marker(
                                type=Marker.TEXT_VIEW_FACING,
                                id = 125,
                                pose=Pose(Point(-5, 0, 0), Quaternion(0, 0, 0, 1)),
                                scale=Vector3(10, 10, 1),
                                header=Header(frame_id='base_link'),
                                text = "%f"%self.lidar_obst_dist,
                                color=ColorRGBA(0.0, 1.0, 0.0, 0.8))

        self.marker_ttc_time_text = Marker(
                                type=Marker.TEXT_VIEW_FACING,
                                id = 126,
                                pose=Pose(Point(-3.6, -3.2, 0), Quaternion(0, 0, 0, 1)),
                                scale=Vector3(10, 10, 1),
                                header=Header(frame_id='base_link'),
                                text = "%f"%self.lidar_obst_dist,
                                color=ColorRGBA(0.8, 0.4, 0.6, 0.8))                                

        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)                                

    def lidar_dist_to_obstacle_callback(self, data):
        self.lidar_obst_dist = data.visible_floor_distance
        rospy.loginfo("lidar_obst_dist : %.2fm" % self.lidar_obst_dist)
        self.update_markers()

    def ttc_callback(self, data):
        self.ttc_time = data.ttc
        self.ttc_azimuth = data.ttc_azimuth
        # self.ttc_obst_dist = float(data.obstacles[0].x)
        rospy.loginfo("ttc : time  %.2fs, azimuth %.2f" % (
                            self.ttc_time, self.ttc_azimuth))
        self.update_markers()

    def update_markers(self) :
        # Marker obstacle
        self.marker_obstacle.points=[Point(0, 0, 0), Point(self.lidar_obst_dist, 0, 0)]        
        self.marker_obstacle.header.stamp = rospy.Time.now()
        self.marker_publisher.publish(self.marker_obstacle)

        # Marker chair (Cube) - Make no change. Maybe can remove this update line ?
        self.marker_chair.header.stamp = rospy.Time.now()
        self.marker_publisher.publish(self.marker_chair)

        # Marker lidar distance Text
        self.marker_dist_text.text = "Visible LIDAR floor distance : %.2fm" % self.lidar_obst_dist
        self.marker_dist_text.header.stamp = rospy.Time.now()
        self.marker_publisher.publish(self.marker_dist_text)

        # Marker ttc time Text
        self.marker_ttc_time_text.text = "TTC time : %.2fs" % self.ttc_time
        self.marker_ttc_time_text.header.stamp = rospy.Time.now()
        self.marker_publisher.publish(self.marker_ttc_time_text)

def my_main(inputs_container):
    # Initialize and run node	
    rospy.init_node('rviz_viz_node')

    lidar_dist_to_obstacle_subscriber = rospy.Subscriber(
                                                        '/wizzy/lidar_proc',
                                                        lidar_data,
                                                        inputs_container.lidar_dist_to_obstacle_callback,
                                                        queue_size=1)

    ttc_subscriber = rospy.Subscriber(
                                        '/ttc',
                                        ttc,
                                        inputs_container.ttc_callback,
                                        queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    inputs_container = CallbackItems()

    try:
        my_main(inputs_container)
    except rospy.ROSInterruptException:
        rospy.logerr(rospy.get_name() + ': Got ROS Interrupt Exception')
    
