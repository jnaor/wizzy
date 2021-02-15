#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from wizzybug_msgs.msg import obstacle, obstacleArray, ChairState

import time
import numpy as np


class ObstacleHandler:
    def __init__(self):
        self.obstacle_subscriber = rospy.Subscriber("/wizzy/obstacle_list", obstacleArray, self.scan_obstacles)
        self.state_publisher = rospy.Publisher('obstacles', ChairState, queue_size=100)

        self.state_object = ChairState()
        self.state = 'WizzyClear'
        self.max_range = 20
        self.range_a = 6
        self.range_b = 4
        self.range_c = 2

    def scan_obstacles(self, obstacle_array):

        min_range = self.max_range  # larger than sensor max range... picked arbitrary
        min_index = 0

        #TODO:
        # Make a list of ALL obstacles within danger zone
        # if not empty - lock wheels
        # Listen to Joystick as well
        # if joystick direction is smaller than a tolerance - stay locked
        # else:
        #   if x direction is larger than tolerance 2:
        #       if joystick dot obstacle < 1/2: release
        #       else: stay locked
        #   else: release (turning around in the spot)

        # Find the closest obstacle:
        for obstacle_id, obstacle_data in enumerate(obstacle_array.data):
            current_range = np.sqrt(obstacle_data.x**2 + obstacle_data**2)
            if current_range < min_range:
                min_range = current_range
                min_index = obstacle_id

        new_state = 'undefined'
        direction = ' '
        # If there's an obstacle within danger range, update state:
        if min_range < self.range_c:
            new_state = 'WizzyC'
            min_obstacle = obstacle_array.data[min_index]
            direction = np.atan2(min_obstacle.y, min_obstacle.x)

        elif min_range < self.range_b:
            new_state = 'WizzyB'

        elif min_range < self.range_a:
            new_state = 'WizzyA'

        else:
            new_state = 'WizzyClear'

        if self.state != new_state:
            self.state = new_state
            print(self.state, direction)
            # Publish state or something


def main():
    # Initialize and run node
    rospy.init_node('obstacle_visualizer')
    obstacle_handler = ObstacleHandler()
    rospy.spin()

    # while not rospy.is_shutdown():
    #     pass


if __name__ == '__main__':
    main()
