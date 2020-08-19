#!/usr/bin/env python
import rospy

# Input measurements from the RPLidar driver node
from sensor_msgs.msg import LaserScan

# Output measurements of hazards to the DM node
from wizzybug_msgs.msg import lidar_data

import numpy as np
from sklearn.linear_model import RANSACRegressor

"""
    Process lidar sensor input from topic /scan, 
    and publish obstacle distance and LidarProcess state (i.e., obstacle type)
    """
class LidarProcess :

    # score threshold for ground plane estimation
    GROUND_PLANE_ESTIMATION_THRESHOLD = 10

    # there's some noise in front of the sensor; estimate ground based on measurements
    # taken after a certain distance
    GROUND_PLANE_ESTIMATION_MIN_DISTANCE = 0.2

    # max distance for ground plane estimation
    GROUND_PLANE_ESTIMATION_MAX_DISTANCE = 0.5

    # maximum difference between successive floor z measurements
    MAX_FLOOR_Z_DIFF = 0.05

    def __init__ (self, min_obstacle_height, min_pitfall_depth):

        # save parameters
        self.min_obstacle_height, self.min_pitfall_depth = min_obstacle_height, min_pitfall_depth

        # initialize publisher 
        self.lidar_proc = rospy.Publisher('/wizzy/lidar_proc', lidar_data, queue_size=10)          

        # Subscribe 
        rospy.Subscriber("/scan", LaserScan, self.scan_cb)

        # simulation-specific correction
        self.simulated_radar = rospy.get_param('simulated_lidar', False)
        
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

        # check that both x and z are not empty
        if len(x) * len(z) == 0:
            rospy.logerr("invalid laser scan readings")
            return

#        from matplotlib import pylab as plt
#        plt.scatter(x, z)
#        plt.show()

        # locations assumed to be the ground
        ground_x, ground_z = x[(x > LidarProcess.GROUND_PLANE_ESTIMATION_MIN_DISTANCE) &
                               (x < LidarProcess.GROUND_PLANE_ESTIMATION_MAX_DISTANCE)], \
                             z[(x > LidarProcess.GROUND_PLANE_ESTIMATION_MIN_DISTANCE) &
                               (x < LidarProcess.GROUND_PLANE_ESTIMATION_MAX_DISTANCE)]

        if min(len(ground_x), len(ground_z)) == 0:
            rospy.logwarn('unable to detect ground readings')
            return

        # estimate ground line using RANSAC
        try:

            ransac = RANSACRegressor(random_state=0).fit(ground_x.reshape(-1, 1), ground_z)

        except ValueError as e:
            rospy.logwarn('RANSAC error {}'.format(e))
            return

        # estimate line at all x
        l = ransac.predict(x.reshape(-1, 1))

        # show
        # show_line_fit(ransac, ground_x, ground_z, l)

        # keep ransac score
        ransac_score = np.abs(ransac.score(ground_x.reshape(-1, 1), ground_z))
        # rospy.logdebug('ransac score is {}'.format(ransac_score))

        # if readings do not fit a line then report error
        if ransac_score > LidarProcess.GROUND_PLANE_ESTIMATION_THRESHOLD:
            rospy.logwarn('unable to detect ground. score is {}'.format(ransac_score))
            return

        # record lidar height (minus sign because the lidar is above the floor)
        ld.lidar_height = -ransac.predict([[0]])[0]

        # floor angle
        inclination = np.arctan((l[0]-l[-1])/(x[-1]-x[0]))
        ld.floor_inclination_degrees = np.rad2deg(inclination)

        # rotation matrix to make ground level
        R = np.array([[np.cos(inclination), -np.sin(inclination)], [np.sin(inclination), np.cos(inclination)]])

        # transform readings
        T = np.matmul(R, np.vstack((x, z)))

        # subtract ground height
        T[1, :] += ld.lidar_height

        # where are there obstacles
        obstacle_loc = np.bitwise_and(T[1, :] > self.min_obstacle_height,
                                      T[0, :] > LidarProcess.GROUND_PLANE_ESTIMATION_MIN_DISTANCE)

        # x's where there is an obstacle
        obstacle_x = T[0, :][obstacle_loc]

        # if none found
        if len(obstacle_x) == 0 :
            obstacle_x = [np.inf]

        # report distance to obstacle
        ld.dist_to_obstacle = min(min(obstacle_x), msg.range_max)

        # where are there pitfalls
        # pitfall_loc = np.bitwise_and(T[1, :] < -self.min_pitfall_depth,
        #                             T[0, :] > LidarProcess.GROUND_PLANE_ESTIMATION_MIN_DISTANCE)

        # x's where there is a pitfall
        #pitfall_x = T[0, :][pitfall_loc]

        # if none found
        #if len(pitfall_x) == 0 :
        #    pitfall_x = [np.inf]

        # ld.dist_to_pitfall = min(min(pitfall_x), msg.range_max)

        # difference between successive z measurements
        diff = np.abs(np.diff(T[1]))

        # detect gap in z
        z_gap = min(np.where(diff > LidarProcess.MAX_FLOOR_Z_DIFF))

        # How far can we see the floor
        if len(z_gap) == 0:
            ld.visible_floor_distance = msg.range_max
            rospy.logwarn('no z gap detected; max range visibility for now ({})'.format(msg.range_max))
        else:
            ld.visible_floor_distance = x[z_gap[0]]

        # publish results
        self.lidar_proc.publish(ld)

        
def polar2cart(rho, phi):
    """
    Helper function - Changing from Polar to Cartesian coordinates
    """
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

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
    myLidarProcess = LidarProcess(min_obstacle_height=0.2, min_pitfall_depth=0.1)
    rospy.spin()
    

