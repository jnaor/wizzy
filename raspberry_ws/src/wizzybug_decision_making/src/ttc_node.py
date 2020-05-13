#!/usr/bin/env python

import numpy as np
import rospy
import copy
from wizzybug_msgs.msg import ttc, lidar_data
from wizzybug_msgs.msg import obstacle, obstacleArray
from geometry_msgs.msg import Twist

# sys.path.append(os.path.join(os.getcwd(), 'devel/lib/python2.7/dist-packages'))
# sys.path.append(os.path.join(os.getcwd(), 'devel/lib/python3/dist-packages'))


# from matplotlib import pyplot as plt

# objects = np.array([[5, 0, 0, 1, 1, 1],
#                     [5, 3, 0, 1, 1, 1],
#                     [5, -3, 0, 1, 1, 1]])

time_step = 0.2
t_horizon = 5 + time_step
robot_radius = 0.4
w_threshold = 0.001
wizzy_width = 0.5
wizzy_length = 0.5
lidar_obj_width = 0.5
lidar_obj_length = 0.5


class Polygon:
    # polygon class for object \ robot modeling
    def __init__(self, x=0.0, y=0.0, yaw=0.0, width=0.0, height=0.0, depth=0.0):
        # polygon object init
        self.pose = {'x': x, 'y': y, 'yaw': yaw}
        self.dimensions = {'width': width, 'height': height, 'depth': depth}
        self.vertices = dict()
        self.get_vertices()
        self.edges = dict()
        self.get_edges()
        self.normals = dict()
        self.get_normals()

    def get_vertices(self):
        # vertices in top view
        self.vertices['bottom_right'] = self.vertex_pose(self.pose, -self.dimensions['depth'] / 2,
                                                         -self.dimensions['width'] / 2)
        self.vertices['top_right'] = self.vertex_pose(self.pose, self.dimensions['depth'] / 2,
                                                      -self.dimensions['width'] / 2)
        self.vertices['bottom_left'] = self.vertex_pose(self.pose, -self.dimensions['depth'] / 2,
                                                        self.dimensions['width'] / 2)
        self.vertices['top_left'] = self.vertex_pose(self.pose, self.dimensions['depth'] / 2,
                                                     self.dimensions['width'] / 2)

    def get_edges(self):
        # edges counter clockwise
        self.edges['right'] = np.array([self.vertices['top_right'][0] - self.vertices['bottom_right'][0],
                                        self.vertices['top_right'][1] - self.vertices['bottom_right'][1]])
        self.edges['top'] = np.array([self.vertices['top_left'][0] - self.vertices['top_right'][0],
                                      self.vertices['top_left'][1] - self.vertices['top_right'][1]])
        self.edges['left'] = np.array([self.vertices['bottom_left'][0] - self.vertices['top_left'][0],
                                       self.vertices['bottom_left'][1] - self.vertices['top_left'][1]])
        self.edges['bottom'] = np.array([self.vertices['bottom_right'][0] - self.vertices['bottom_left'][0],
                                         self.vertices['bottom_right'][1] - self.vertices['bottom_left'][1]])

    def update_pose(self, x, y, yaw):
        self.pose['x'] = x
        self.pose['y'] = y
        self.pose['yaw'] = yaw
        self.get_vertices()
        self.get_edges()
        self.get_normals()

    def get_normals(self):
        # right normals \ outward normals
        for key in self.edges:
            self.normals[key] = np.array([self.edges[key][1], -self.edges[key][0]])
            self.normals[key] /= np.linalg.norm(self.normals[key])

    def calc_min_max_along_vec(self, vec):
        min_proj = 1000.0
        max_proj = -1000.0
        for vertex in self.vertices.values():
            projection = np.dot(vertex, vec)
            if projection > max_proj:
                max_proj = projection
            if projection < min_proj:
                min_proj = projection
        return max_proj, min_proj

    def is_colliding(self, adverse_poly):
        for normal in self.normals.values():
            max_self, min_self = self.calc_min_max_along_vec(normal)
            max_adverse, min_adverse = adverse_poly.calc_min_max_along_vec(normal)
            if max_self < min_adverse or max_adverse < min_self:
                return False

        for normal in adverse_poly.normals.values():
            max_self, min_self = self.calc_min_max_along_vec(normal)
            max_adverse, min_adverse = adverse_poly.calc_min_max_along_vec(normal)
            if max_self < min_adverse or max_adverse < min_self:
                return False

        return True

    def diff_robot_move(self, v, w, dt):

        if w < w_threshold:
            x_new = self.pose['x'] + v * dt * np.cos(self.pose['yaw'])
            y_new = self.pose['y'] + v * dt * np.sin(self.pose['yaw'])
            yaw_new = self.pose['yaw']

        else:
            R = v / w
            x_new = np.cos(w * dt) * np.sin(self.pose['yaw']) * R + np.sin(w * dt) * np.cos(self.pose['yaw']) * R + \
                    self.pose['x'] - np.sin(self.pose['yaw']) * R
            y_new = np.sin(w * dt) * np.sin(self.pose['yaw']) * R - np.cos(w * dt) * np.cos(self.pose['yaw']) * R + \
                    self.pose['x'] + np.cos(self.pose['yaw']) * R
            yaw_new = self.pose['yaw'] + dt * w

        self.update_pose(x_new,y_new, yaw_new)

    @staticmethod
    def vertex_pose(self_pose, dx, dy):
        vertex_pose = np.zeros((2), dtype=np.float64)
        vertex_pose[0] = np.cos(self_pose['yaw']) * dx + np.sin(self_pose['yaw']) * dy + self_pose['x']
        vertex_pose[1] = -np.sin(self_pose['yaw']) * dx + np.cos(self_pose['yaw']) * dy + self_pose['y']
        return vertex_pose


def calc_time_to_collision(v, w, objects, lidar_dist):


    wizzy = Polygon(width=wizzy_width, depth=wizzy_length)
    lidar_obj = Polygon(x=lidar_dist, width=wizzy_width, depth=wizzy_length)
    ttc = t_horizon
    ttc_azimuth = 0.0
    for i in range(int(t_horizon / time_step)):

        wizzy.diff_robot_move(v, w, time_step)

        if wizzy.is_colliding(lidar_obj):
            if ttc > time_step * i:
                ttc = time_step * i
                ttc_azimuth = 0.0

        for obj in objects:
            obj_poly = Polygon(x = obj.x, y = obj.y, yaw = 0.0, width = obj.width, depth = obj.length)
            if wizzy.is_colliding(obj_poly):
                if ttc > time_step * i:
                    ttc = time_step * i
                    ttc_azimuth = np.arctan2(obj.y.data, obj.x.data)

    return ttc, ttc_azimuth


class CallbackItems:

    def __init__(self):
        self.v = 0.0
        self.w = 0.0
        self.lidar_dist = 0.0
        self.objects = []
        self.ttc_msg = ttc()

    def lidar_dist_to_obstacle_callback(self, data):
        self.lidar_dist = min([data.dist_to_pitfall, data.dist_to_obstacle])

    def joy_callback(self, data):
        self.v = data.linear.x
        self.w = data.angular.z

    def objects_sub_callback(self, data):
        lidar_obj = obstacle()
        lidar_obj.y = 0.0
        lidar_obj.x = self.lidar_dist
        lidar_obj.width = wizzy_width
        lidar_obj.length = wizzy_length
        self.objects = data.data


if __name__ == '__main__':
    rospy.init_node('ttc_node')
    #
    ttc_msg = ttc()
    lidar_obj = obstacle()
    obs_msg = obstacleArray()
    #
    inputs_container = CallbackItems()
    obj_sub = rospy.Subscriber('wizzy/obstacle_list', obstacleArray, inputs_container.objects_sub_callback)
    lidar_dist_to_obstacle_subscriber = rospy.Subscriber('/wizzy/lidar_proc', lidar_data, inputs_container.lidar_dist_to_obstacle_callback)
    joy_sub = rospy.Subscriber('/cmd_vel', Twist, inputs_container.joy_callback)
    #
    ttc_pub = rospy.Publisher('/ttc', ttc, queue_size=10)
    # loop rate of 8Hz
    rate = rospy.Rate(8)

    while not rospy.is_shutdown():
        #
        objects_temp = copy.deepcopy(inputs_container.objects)
        lidar_temp = copy.deepcopy(inputs_container.lidar_dist)
        t, ang = calc_time_to_collision(inputs_container.v, inputs_container.w, objects_temp, lidar_temp)
        #
        ttc_msg.ttc = t
        ttc_msg.ttc_azimuth = ang
        ttc_msg.header.stamp = rospy.Time.now()
        lidar_obj.y.data = 0.0
        lidar_obj.x.data = lidar_temp
        lidar_obj.width.data = wizzy_width
        lidar_obj.length.data = wizzy_length
        objects_temp.append(lidar_obj)
        # obs_msg.data = objects_temp
        ttc_msg.obstacles = objects_temp
        ttc_pub.publish(ttc_msg)
        # print(ttc_msg)
        rate.sleep()
