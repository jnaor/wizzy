#!/usr/bin/env python

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3
from wizzybug_msgs.msg import lidar_data, obstacle, obstacleArray

if __name__ == "__main__":

    # initialize node
    rospy.init_node('sensing_visualization')

    # subscribe to lidar
    lidar_subscriber = rospy.Subscriber('/wizzy/lidar_proc', ChairState, handler.activation_callback)
    
    # subscrive to camera results
    camera_subscriber = rospy.Subscriber('/wizzy/obstacle_list', ChairState, handler.activation_callback)
    motors_publisher = rospy.Publisher('hmi_motors_array', MarkerArray, queue_size = 10)
    leds_publisher = rospy.Publisher('hmi_leds_array', MarkerArray, queue_size = 10)