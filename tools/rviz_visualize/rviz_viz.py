#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

import math
import time

# To solve frame TF issue, need to run :
# $ rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map base_link 10

def show_text_in_rviz(marker_publisher, text):
    marker = Marker(
#                type=Marker.TEXT_VIEW_FACING,
                type=Marker.SPHERE,
                id=0,
                action=Marker.ADD,
                lifetime=rospy.Duration(1.5),
                pose=Pose(Point(1,1,1), Quaternion(0, 0, 0, 1)),
                scale=Vector3(1, 1, 1),
                header=Header(frame_id='base_link'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
    
    marker_publisher.publish(marker)

def my_main():
  """
  One place to initalize and run the code
  """
  # Initialize and run node	
  rospy.init_node('my_node')
  marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)

  cnt = 0
  while (1) :
    if cnt > 10 : cnt = 0
    rospy.sleep(0.5)                                                             
    show_text_in_rviz(marker_publisher, 'Hello world! ' + str(cnt))
    cnt += 1

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
  idan_main()
