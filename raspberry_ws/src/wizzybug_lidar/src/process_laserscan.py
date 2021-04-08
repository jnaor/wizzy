#!/usr/bin/env python
import rospy

# Input measurements from the RPLidar driver node
from sensor_msgs.msg import LaserScan

# Output measurements of hazards to the DM node
from wizzybug_msgs.msg import lidar_data

import numpy as np
from sklearn.linear_model import RANSACRegressor
from scipy.ndimage.morphology import binary_erosion

# processing rate (in Hertz)
LIDAR_PROCESS_RATE = 10

"""
    Process lidar sensor input from topic /scan, 
    and publish obstacle distance and LidarProcess state (i.e., obstacle type)
    """


class LidarProcess:
    # score threshold for ground plane estimation
    GROUND_PLANE_ESTIMATION_THRESHOLD = 10

    # there's some noise in front of the sensor; estimate ground based on measurements
    # taken after a certain distance
    GROUND_PLANE_ESTIMATION_MIN_DISTANCE = 0.0

    # max distance for ground plane estimation
    GROUND_PLANE_ESTIMATION_MAX_DISTANCE = 0.3

    # maximum difference between successive floor measurements
    MAX_FLOOR_X_DIFF, MAX_HEIGHT_ABOVE_FLOOR = 0.5, 0.5 # 0.3, 0.2

    def __init__(self, min_obstacle_height, min_pitfall_depth, visualize=False):

        # TODO: remove
        np.set_printoptions(precision=2)

        # save parameters
        self.min_obstacle_height, self.min_pitfall_depth = min_obstacle_height, min_pitfall_depth

        # initialize publisher 
        self.lidar_proc = rospy.Publisher('/wizzy/lidar_proc', lidar_data, queue_size=10)

        # Subscribe 
        rospy.Subscriber("/scan", LaserScan, self.scan_cb)

        # simulation-specific correction
        self.simulated_radar = rospy.get_param('simulated_lidar', False)

        # visualization on/off
        self.visualize = visualize

        # no messages received so far
        self.current_message = None

    def scan_cb(self, msg):
        # save as current message
        self.current_message = msg

    def process_scan(self):

        # if there are no laser scans bail out
        if self.current_message is None:
            rospy.logwarn('no lidar scans received')
            return

        # to hold message to be published
        ld = lidar_data()

        # get last message
        msg = self.current_message

        # copy range readings from message to numpy array
        range_readings = np.array(msg.ranges)

        # get angle ranges from message
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)

        # patch warning: for some reason the simulation and physical units return messages of different sizes
        min_length = min(len(angles), len(range_readings))
        range_readings = range_readings[:min_length]
        angles = angles[:min_length]

        # polar to Cartesian coordinates change. Notice x, z flipping (because angle is measured towards the z axis,
        # and not towards x axis)
        z, x = polar2cart(range_readings, angles)

        # TODO: ugly patch!!
        if self.simulated_radar:
            x = -x

        else:
            z = -z

        # restrict to places where we have finite readings
        finite_indices = np.isfinite(x) & np.isfinite(z)
        x, z = x[finite_indices], z[finite_indices]

        # sort by x
        sorted_indices = np.argsort(x)
        x, z = x[sorted_indices], z[sorted_indices]

        # check that both x and z are not empty
        if len(x) * len(z) == 0:
            rospy.logerr("invalid laser scan readings")
            return

        # locations assumed to be the ground
        ground_indices = np.where((x > LidarProcess.GROUND_PLANE_ESTIMATION_MIN_DISTANCE) &
                                  (x < LidarProcess.GROUND_PLANE_ESTIMATION_MAX_DISTANCE))
        ground_x, ground_z = x[ground_indices], z[ground_indices]

        if min(len(ground_x), len(ground_z)) == 0:
            rospy.logwarn('unable to detect ground readings')
            return

        # estimate ground line using RANSAC
        try:
            ransac = RANSACRegressor(random_state=0).fit(ground_x.reshape(-1, 1), ground_z)

        except ValueError as e:
            rospy.logwarn('RANSAC error {}'.format(e))
            return

        # keep ransac score
        ransac_score = np.abs(ransac.score(ground_x.reshape(-1, 1), ground_z))

        # if readings do not fit a line then report error
        if ransac_score > LidarProcess.GROUND_PLANE_ESTIMATION_THRESHOLD:
            rospy.logwarn('unable to detect ground. score is {}'.format(ransac_score))
            return

        # estimate line at all x's
        l = ransac.predict(x.reshape(-1, 1))

        # record lidar height (minus sign because the lidar is above the floor)
        ld.lidar_height = -ransac.predict([[0]])[0]

        # floor angle
        inclination = np.arctan((l[0] - l[-1]) / (x[-1] - x[0]))
        ld.floor_inclination_degrees = np.rad2deg(inclination)

        # rotation matrix to make ground level
        R = np.array([[np.cos(inclination), -np.sin(inclination)], [np.sin(inclination), np.cos(inclination)]])

        # transform readings
        T = np.matmul(R, np.vstack((x, z)))

        # subtract ground height
        T[1, :] += ld.lidar_height

        # get real heights (after the rotations, etc.)
        heights = T[1, :]

        # disregard the ground indices
        # if something bad happened there then hopefully we'll see it in the ransac score
        heights[ground_indices] = 0

        # difference between successive x measurements. stick a zero at the beginning to preserve the length of the
        # vector
        diff_x = np.append([0], np.abs(np.diff(T[0, :])))

        # where are there either obstacles or gaps in X or large heights
        bad_indices = np.where(np.bitwise_or(heights > LidarProcess.MAX_HEIGHT_ABOVE_FLOOR,
                                             diff_x > LidarProcess.MAX_FLOOR_X_DIFF))[0]

        # if none found
        if len(bad_indices) == 0:
            ld.visible_floor_distance = 1.0

        else:

            # find first place with obstacles or gaps
            first_idx = max([0, np.min(bad_indices)-1])

            # floor is visible where there is no large gap in x and no obstacle in z
            ld.visible_floor_distance = T[0][first_idx]

        if self.visualize:
            from matplotlib import pylab as plt
            from drawnow import drawnow, figure

            def visualize():
                #plt.figure(1)
                # plt.subplot(1, 1, 1)
                #plt.scatter(x, z)
                #plt.scatter(ground_x, ground_z, color='blue')
                #plt.plot(x, ransac.predict(x.reshape(-1, 1)), color='red')

                #plt.figure(1)
                # plt.subplot(1, 1, 1)
                #plt.plot(range_readings)

                #plt.figure(2)
                #plt.plot(angles)
                #plt.plot(x, ransac.predict(x.reshape(-1, 1)), color='red')

                plt.figure(1)
                # plt.subplot(1, 1, 1)
                plt.scatter(T[0, :], T[1, :])
                # plt.scatter(x[1:], diff_z, color='green')
                # plt.scatter(T[0], T[1], color='green')
                plt.plot(ld.visible_floor_distance, 0, 'bx')

                plt.xlim([-0.5, 6])
                plt.ylim([-1, 3])

            drawnow(visualize)

        # publish results
        print("FLoor : " + str(ld.visible_floor_distance))
        self.lidar_proc.publish(ld)


def polar2cart(rho, phi):
    """
    Helper function - Changing from Polar to Cartesian coordinates
    """
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return x, y


if __name__ == '__main__':
    rospy.init_node('Lidar_process', log_level=rospy.INFO)

    # TODO: read from json or something
    myLidarProcess = LidarProcess(min_obstacle_height=0.2, min_pitfall_depth=0.1, visualize=False)

    # rate to perform calculation
    rate = rospy.Rate(LIDAR_PROCESS_RATE)

    while not rospy.is_shutdown():
        # do processing and publishing
        myLidarProcess.process_scan()

        # sleep
        rate.sleep()

    rospy.spin()
