#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
import time
import math

def main():
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

    while not rospy.is_shutdown():
        marker.scale.x = 2 + math.sin(time.time())
        marker.header.stamp = rospy.Time.now()
        marker_publisher.publish(marker)
        rospy.sleep(0.5)


if __name__ == '__main__':
    main()
