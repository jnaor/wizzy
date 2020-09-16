#!/usr/bin/env python
import rospy

# Input measurements from the RPLidar driver node
from sensor_msgs.msg import LaserScan

# Output measurements of hazards to the DM node
from wizzybug_msgs.msg import lidar_data

import numpy as np
from sklearn.linear_model import RANSACRegressor
from scipy.ndimage.morphology import binary_erosion

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
    MAX_FLOOR_X_DIFF, MAX_FLOOR_Z_DIFF = 0.3, 0.3

    def __init__(self, min_obstacle_height, min_pitfall_depth, visualize=False):

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

    def scan_cb(self, msg):

        # prepare result structure to publish
        ld = lidar_data()

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

        # estimate line at ground_x
        l = ransac.predict(ground_x.reshape(-1, 1))

        # keep ransac score
        ransac_score = np.abs(ransac.score(ground_x.reshape(-1, 1), ground_z))

        # if readings do not fit a line then report error
        if ransac_score > LidarProcess.GROUND_PLANE_ESTIMATION_THRESHOLD:
            rospy.logwarn('unable to detect ground. score is {}'.format(ransac_score))
            return

        # ransac inliers
        inlier_mask = ransac.inlier_mask_

        # ransac outliers
        outlier_mask = np.logical_not(inlier_mask)

        # find index of last outlier in original array
        try:
            last_outlier_index = ground_indices[0][np.max(np.nonzero(outlier_mask)[0])]

        # if there are no outliers
        except ValueError:
            # take last ground point
            last_outlier_index = 0

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

        # difference between successive z measurements
        diff_x, diff_z = np.abs(np.diff(T[0])), np.abs(np.diff(T[1]))

        # skip to after the last outlier
        diff_z[:last_outlier_index+1] = 0

        # x gaps
        x_gap = binary_erosion(diff_x < LidarProcess.MAX_FLOOR_X_DIFF)

        # detect gap in z
        try:
            z_gap = min(np.where(np.bitwise_and(x_gap, diff_z > LidarProcess.MAX_FLOOR_Z_DIFF))[0])
            ld.visible_floor_distance = x[z_gap - 1]
        except ValueError:
            ld.visible_floor_distance = np.max(x)
            # rospy.logwarn('no z gap detected; max range visibility for now ({})'.format(msg.range_max))

        if self.visualize:
            from matplotlib import pylab as plt
            from drawnow import drawnow, figure

            def visualize():
                # plt.figure(1)
                # plt.subplot(1, 1, 1)
                # plt.scatter(x, z)
                # # plt.scatter(ground_x, ground_z)
                #
                #
                # plt.scatter(ground_x[inlier_mask], ground_z[inlier_mask], color='yellowgreen', marker='.',
                #             label='Inliers')
                # plt.scatter(ground_x[outlier_mask], ground_z[outlier_mask], color='black', marker='.',
                #             label='Outliers')
                #

                # plt.figure(2)
                plt.subplot(1, 1, 1)
                # plt.scatter(T[0, :], T[1, :])
                plt.scatter(x[1:], diff_z, color='green')
                plt.plot(ld.visible_floor_distance, 0, 'bx')

                plt.xlim([-0.5, 6])
                plt.ylim([-1, 3])

            drawnow(visualize)


        # publish results
        self.lidar_proc.publish(ld)


def polar2cart(rho, phi):
    """
    Helper function - Changing from Polar to Cartesian coordinates
    """
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return (x, y)


def show_line_fit(ransac, X, y, y_ransac):
    from matplotlib import pylab as plt

    inlier_mask = ransac.inlier_mask_
    outlier_mask = np.logical_not(inlier_mask)

    lw = 2
    plt.clf()
    plt.scatter(X[inlier_mask], y[inlier_mask], color='yellowgreen', marker='.',
                label='Inliers')
    plt.scatter(X[outlier_mask], y[outlier_mask], color='cornflowerblue', marker='.',
                label='Outliers')
    # plt.plot(X, y, color='navy', linewidth=lw, label='Linear regressor')
    # plt.plot(X, y_ransac, color='cornflowerblue', linewidth=lw,
    #          label='RANSAC regressor')
    # plt.legend(loc='lower right')
    # plt.xlabel("Input")
    # plt.ylabel("Response")
    plt.show(0.2)


if __name__ == '__main__':
    rospy.init_node('Lidar_process', log_level=rospy.DEBUG)

    # TODO: read from json or something
    myLidarProcess = LidarProcess(min_obstacle_height=0.2, min_pitfall_depth=0.1, visualize=False)
    rospy.spin()
