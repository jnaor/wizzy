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

    # meters ahead used for ground plane estimate
    GROUND_PLANE_ESTIMATION_DISTANCE = 0.5

    # score threshold for ground plane estimation
    GROUND_PLANE_ESTIMATION_THRESHOLD = 0.5

    def __init__ (self, min_obstacle_height, min_pitfall_depth):

        # save parameters
        self.min_obstacle_height, self.min_pitfall_depth = min_obstacle_height, min_pitfall_depth

        # initialize publisher 
        self.lidar_proc = rospy.Publisher('/wizzy/lidar_proc', lidar_data, queue_size=10)       

        # Debug - Show the laser scan we ar working on         
        self.lidar_proc_raw = rospy.Publisher('/wizzy/lidar_proc_raw', LaserScan, queue_size=10)       

        # Subscribe 
        rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        
    def scan_cb(self, msg):           

        # prepare result structure to publish
        ld = lidar_data()

        # copy range readings from message to numpy array
        range_readings = np.array(msg.ranges)

        # get angle ranges from message
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)

        # polar to Cartesian coordinates change. Notice x, z flipping (because angle is measured towards the z axis,
        # and not towards x axis)
        z, x = polar2cart(msg.ranges[:-1], angles)
        x = -x

        # restrict to places where we have finite readings
        finite_indices = np.isfinite(x) & np.isfinite(z)
        x, z = x[finite_indices], z[finite_indices]

        # check that both x and z are not empty
        if len(x) * len(z) == 0:
            rospy.logerr("invalid laser scan readings")
            return

        # locations assumed to be the ground
        ground_x, ground_z = x[(x > 0) & (x < 0.5)], z[(x > 0) & (x < 0.5)]

        # estimate ground line using RANSAC
        ransac = RANSACRegressor(random_state=0).fit(ground_x.reshape(-1, 1), ground_z)

        # if readings do not fit a line then report error
        if np.abs(ransac.score(ground_x.reshape(-1, 1), ground_z)) > LidarProcess.GROUND_PLANE_ESTIMATION_THRESHOLD:
            rospy.logwarn('unable to detect ground')
            ld.dist_to_obstacle = 0
            ld.dist_to_pitfall = 0
            ld.visible_floor_distance = 0
            self.lidar_proc.publish(ld)
            return

        # estimate line at all x
        l = ransac.predict(x.reshape(-1, 1))

        # record lidar height (minus sign because the lidar is above the floor)
        ld.lidar_height = -ransac.predict([[0]])[0]

        # floor angle
        ld.floor_inclination_degrees = 180 - np.rad2deg(np.arctan2(l[-1]-l[0], x[-1]-x[0]))

        # x's where there is an obstacle
        obstacle_x = x[l + self.min_obstacle_height < z]

        # TODO: get rid of the second condition here (probably the result of the chair interfering with the readings,
        #  but this hasn't been checked)
        try:
            ld.dist_to_obstacle = min(obstacle_x[np.abs(obstacle_x) > LidarProcess.GROUND_PLANE_ESTIMATION_DISTANCE])
        except ValueError:
            ld.dist_to_obstacle = msg.range_max

        # x's where there is a pitfall
        pitfall_x = x[l - self.min_pitfall_depth > z]

        # TODO: same goes for distance to pitfall
        try:
            ld.dist_to_pitfall = min(pitfall_x[np.abs(pitfall_x) > LidarProcess.GROUND_PLANE_ESTIMATION_DISTANCE])
        except ValueError:
            ld.dist_to_pitfall = msg.range_max

        # How far can we see the floor
        ld.visible_floor_distance = np.max(x)

        # publish results
        self.lidar_proc.publish(ld)

        
def polar2cart(rho, phi):
    """
    Helper function - Changing from Polar to Cartesian coordinates
    """
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

    
if __name__ == '__main__':
    rospy.init_node('Lidar_process', log_level=rospy.DEBUG)

    # TODO: read from json or something
    myLidarProcess = LidarProcess(min_obstacle_height=0.2, min_pitfall_depth=0.05)
    rospy.spin()
    

