#!/usr/bin/env python

import rospy

from visualization_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from wizzybug_msgs.msg import obstacle, obstacleArray
import numpy as np
from tf.transformations import euler_from_quaternion
import time

# for how long to display markers
MARKER_DISPLAY_DURATION = 0.1


class ObstacleSim:

    def __init__(self):
        
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        self.obstacle_publisher = rospy.Publisher('/wizzy/obstacle_list', obstacleArray, queue_size=100)

        self.absolute_obstacles = []
        for idx in range(3):
            self.absolute_obstacles.append(Vector3())

        self.absolute_obstacles[0].x = 4
        self.absolute_obstacles[0].y = 0
        self.absolute_obstacles[0].z = 0.1

        self.absolute_obstacles[1].x = 0
        self.absolute_obstacles[1].y = 4
        self.absolute_obstacles[1].z = 1

        self.absolute_obstacles[2].x = 0
        self.absolute_obstacles[2].y = -4
        self.absolute_obstacles[2].z = 1

        self.obstacle_array = obstacleArray()
        for current_obstacle in self.absolute_obstacles:
            new_obstacle = obstacle()
            new_obstacle.x = current_obstacle.x
            new_obstacle.y = current_obstacle.y
            new_obstacle.z = current_obstacle.z
            self.obstacle_array.data.append(new_obstacle)

        # print(self.obstacle_array)

    def odometry_callback(self, odom_data):
        robot_pos_x = odom_data.pose.pose.position.x
        robot_pos_y = odom_data.pose.pose.position.y
        quat = odom_data.pose.pose.orientation
        robot_yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]

        for obstacle_idx, obstacle_data in enumerate(self.absolute_obstacles):
            distance = np.sqrt((obstacle_data.x - robot_pos_x)**2 + (obstacle_data.y - robot_pos_y)**2)
            angle = np.arctan2(obstacle_data.y - robot_pos_y, obstacle_data.x - robot_pos_x)
            self.obstacle_array.data[obstacle_idx].x = distance * np.cos(angle-robot_yaw)
            self.obstacle_array.data[obstacle_idx].y = distance * np.sin(angle-robot_yaw)
            # if obstacle_idx == 0:
            #     print(self.obstacle_array.data[0].x,
            #           self.obstacle_array.data[0].y,
            #           distance,
            #           angle,
            #           angle-robot_yaw)
            #     print(' ')

    def publish_array(self):
        self.obstacle_publisher.publish(self.obstacle_array)
        # print('made it')


if __name__ == "__main__":
    # initialize node
    rospy.init_node('obstacle_simulator')

    obstacle_handler = ObstacleSim()

    while not rospy.is_shutdown():
        obstacle_handler.publish_array()
        time.sleep(0.1)







