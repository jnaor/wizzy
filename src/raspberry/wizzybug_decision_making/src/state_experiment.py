#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, Twist
from std_msgs.msg import Header, ColorRGBA, String
from wizzybug_msgs.msg import obstacle, obstacleArray, ChairState

import time
import numpy as np


class ObstacleHandler:
    def __init__(self):
        self.obstacle_subscriber = rospy.Subscriber("/wizzy/obstacle_list", obstacleArray, self.scan_obstacles)
        self.joystick_subscriber = rospy.Subscriber("/cmd_vel", Twist, self.update_movement)
        self.relay_publisher = rospy.Publisher('usb_relay_command', String, queue_size=10)
        # self.state_publisher = rospy.Publisher('chair_state', ChairState, queue_size=100)

        self.state_object = ChairState()
        self.state = 'WizzyClear'
        self.locked = False
        self.linear = 0
        self.angular = 0

        self.MAX_RANGE = 20.0
        self.RANGE_A = 6.0
        self.RANGE_B = 4.0
        self.RANGE_C = 2.0
        self.X_TOLERANCE = 0.1

        self.DEBUG = True
        self.marker_publisher = rospy.Publisher('state_debug', MarkerArray, queue_size=100)
        self.debug_markers = MarkerArray()
        for idx in range(3):
            new_marker = Marker()
            new_marker.pose.orientation = Quaternion(0, 0, 0, 1)
            new_marker.header.frame_id = "base_footprint"
            new_marker.id = 297 + idx
            self.debug_markers.markers.append(new_marker)

        self.lock_marker = self.debug_markers.markers[0]
        self.text_marker = self.debug_markers.markers[1]
        self.direction_marker = self.debug_markers.markers[2]

        self.lock_marker.type = Marker.SPHERE
        self.lock_marker.scale = Vector3(0.5, 0.5, 0.5)
        self.lock_marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)
        self.lock_marker.pose.position = Point(0.0, 0.0, 1.0)

        self.text_marker.type = Marker.TEXT_VIEW_FACING
        self.text_marker.scale.z = 0.3
        self.text_marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        self.text_marker.pose.position = Point(0.0, 0.0, 1.5)
        self.text_marker.text = self.state

        self.direction_marker.type = Marker.ARROW
        self.direction_marker.scale = Vector3(0.1, 0.2, 0.0)
        self.direction_marker.color = ColorRGBA(0.0, 1.0, 1.0, 1.0)
        self.direction_marker.pose.position = Point(0.0, 0.0, 0.0)
        self.direction_marker.points.append(Point(0.0, 0.0, 0.0))
        self.direction_marker.points.append(Point(1.0, 0.0, 0.0))


    def scan_obstacles(self, obstacle_array):

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

        obstacles_a = []
        obstacles_b = []
        obstacles_c = []
        new_state = 'undefined'
        movement_direction = 0.0
        obstacle_direction = 0.0
        similarity = 0.0

        # Categorize obstacles:
        for obstacle_id, obstacle_data in enumerate(obstacle_array.data):
            current_range = np.sqrt(obstacle_data.x**2 + obstacle_data.y**2)
            if current_range < self.RANGE_C:
                obstacles_c.append(obstacle_data)

            elif current_range < self.RANGE_B:
                obstacles_b.append(obstacle_data)

            elif current_range < self.RANGE_A:
                obstacles_a.append(obstacle_data)

        # Make decisions based on distances.
        # If there are obstacles in the danger zone:
        if obstacles_c:
            new_state = 'WizzyC'
            if abs(self.linear) < self.X_TOLERANCE:
                # Allow spinning in place.
                if abs(self.angular) > 0:
                    self.unlock_movement()
                else:
                    self.lock_movement()
            else:
                # If there is movement forward, lock if its heading towards an obstacle.
                movement_direction = np.array([self.linear, self.angular])
                movement_norm = np.linalg.norm(movement_direction)
                if movement_norm > 0.01:
                    movement_direction = movement_direction / movement_norm
                all_clashes = []
                for curr_obs in obstacles_c:
                    obstacle_direction = np.array([curr_obs.x, curr_obs.y])
                    obstacle_norm = np.linalg.norm(obstacle_direction)
                    if obstacle_norm > 0.01:
                        obstacle_direction = obstacle_direction / obstacle_norm
                    similarity = np.dot(obstacle_direction, movement_direction)
                    # print(obstacle_direction, obstacle_norm, movement_direction, similarity)
                    all_clashes.append(similarity < 0.5)
                if all(all_clashes):
                    self.unlock_movement()
                else:
                    self.lock_movement()


        else:
            if obstacles_b:
                new_state = 'WizzyB'
            elif obstacles_a:
                new_state = 'WizzyA'
            else:
                new_state = 'WizzyClear'

            self.unlock_movement()

        if self.state != new_state:
            self.state = new_state
            # print(self.state, movement_direction)
            # Publish state or something

        if self.DEBUG:
            self.text_marker.text = self.state
            self.direction_marker.points[1] = Point(self.linear * 2, self.angular * 2, 0.0)
            timestamp = rospy.Time.now()
            for current_marker in self.debug_markers.markers:
                current_marker.header.stamp = timestamp
            self.marker_publisher.publish(self.debug_markers)
            # print(obstacle_direction, movement_direction, similarity)

    def update_movement(self, data):
        self.linear = data.linear.x
        self.angular = data.angular.z

    def lock_movement(self):
        if not self.locked:
            self.locked = True
            self.lock_marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
            self.relay_publisher.publish(String('on'))

    def unlock_movement(self):
        if self.locked:
            self.locked = False
            self.lock_marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)
            self.relay_publisher.publish(String('off'))


def main():
    # Initialize and run node
    rospy.init_node('state_experiment')
    obstacle_handler = ObstacleHandler()
    rospy.spin()

    # while not rospy.is_shutdown():
    #     pass


if __name__ == '__main__':
    main()
