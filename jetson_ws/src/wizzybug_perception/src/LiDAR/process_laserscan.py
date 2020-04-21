#!/usr/bin/env python
import rospy

from std_msgs.msg       import Float64

# Input measurements from the RPLidar driver node
from sensor_msgs.msg    import LaserScan

# Output measurements of hazards to the DM node
from wizzybug_msgs.msg  import lidar_data

import numpy as np


class LidarProcess :
    """
    Process lidar sensor input from topic /scan, 
    and publish obstacle distance and LidarProcess state (i.e., obstacle type)
    """
    def __init__ (self) :
        # initialize publisher 
        self.lidar_proc = rospy.Publisher ('/wizzy/lidar_proc', lidar_data, queue_size=10)       
        
        # Subscribe 
        rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        
    def scan_cb(self, msg):           
        # Set parameters
        # ---------------
        # -178.0 deg. -180deg is straight down from the lidar.
        # It is turning counter clock wise            
        START_WIN_ANG   = (-178)                                         
        END_WIN_ANG     = (-70)         # -70.0 deg                                                                                                      
        
        # 5cm depth allowed at most 
        BELOW_GND_HEIGHT_LIMIT = (-0.05)
        
        # 20cm is the maximum object height above ground to stop wheel chair        
        ABOVE_GND_HEIGHT_LIMIT = (0.20)          
        
        # Number of readings to base ground plane inclination estimate 
        # 30 measurement points from the beginning of measurement is about 12cm 
        # distance from the lidar
        # TODO: kind of a lame way to do this
        GROUND_PLANE_ESTIMATE_MAX_INDEX = 30
        
        # Points with height less then 40cm will be considered part of the floor
        FLOOR_MAX_HEIGHT = (0.4)
    
        # Lidar maximum detection distance - 20m for RPLidar A2
        MAX_LIDAR_DISTANCE = (20)
       
        # Measurements validity check 
        # ---------------------------
        if np.rad2deg(msg.angle_min) > START_WIN_ANG or \
           np.rad2deg(msg.angle_max) < END_WIN_ANG :
               raise ValueError ("Scan vector do not contain the relevant window values")

        # Capture relevant slice for the filters to work on - Only what
        # is in-front of the lidar
        msg.ranges = msg.ranges[180 + START_WIN_ANG: 180 + END_WIN_ANG]  
                              
        # Polar to Cartesian coordinates change
        # Notice x, y flipping (because angle is measured towards the y axis, and not towards x axis)
        y, x = polar2cart(msg.ranges, np.deg2rad(range(START_WIN_ANG, END_WIN_ANG)))

        # Restrict x and y to finite elements
        x, y = x[np.where(np.bitwise_and(np.isfinite(x), np.isfinite(y)))], \
               y[np.where(np.bitwise_and(np.isfinite(x), np.isfinite(y)))]

        # check that both x and y are not empty
        if len(x) * len(y) == 0:
            rospy.logerr("invalid laser scan readings")
            return

        # Fit a line to first measurements which are nearest to the lidar, towards the floor
        l = np.polyfit(x[:GROUND_PLANE_ESTIMATE_MAX_INDEX], y[:GROUND_PLANE_ESTIMATE_MAX_INDEX], 1)

        # Angle of floor (should ideally have been zero but isn't). in radians
        floor_angle = np.arctan(l[0])

        # Rotate all measurements so that y is parallel to floor, and move so that origin is zero
        T = np.array([[np.cos(floor_angle), -np.sin(floor_angle), 0], 
                      [np.sin(floor_angle), np.cos(floor_angle), 0], [0, 0, 1]])

        # Perform transformation
        N = np.matmul (np.hstack((x[:, np.newaxis], y[:, np.newaxis], np.ones((y.shape[0], 1)))), T)

        # de-homogenize
        N = N[:, :2] / N[:, [-1]]

        # Estimate "bias" - height of LIDAR
        lidar_height = np.mean(N[:GROUND_PLANE_ESTIMATE_MAX_INDEX, 1])

        # Bring measurements to around zero
        N[:, 1] -= lidar_height 

        # try to make less ugly
        distances, heights = N[:, 0], N[:, 1]

        # Run Filters
        # -----------
        # Indices where bad things happen
        below_ground_indices = np.where(heights < BELOW_GND_HEIGHT_LIMIT)[0]
        above_ground_indices = np.where(heights > ABOVE_GND_HEIGHT_LIMIT)[0]  

        #  Initialize hazards distance to maximum value possible
        dist_to_pitfall, dist_to_obstacle = MAX_LIDAR_DISTANCE, MAX_LIDAR_DISTANCE

        # Check if there is a pitfall within range
        if len(below_ground_indices) > 0: 
            dist_to_pitfall = abs(distances[below_ground_indices[0]])

        # Check for obstacles above ground within range   
        if len(above_ground_indices) > 0:
            dist_to_obstacle = abs(distances[above_ground_indices[0]])

        # How far can we see the floor
        dist_of_floor = np.max(abs(distances[np.where(abs(heights) <= FLOOR_MAX_HEIGHT)]))
        
        # Publish results
        # --------------- 
        ld = lidar_data()
        ld.dist_to_pitfall = dist_to_pitfall
        ld.dist_to_obstacle = dist_to_obstacle
        ld.dist_to_floor = dist_of_floor        
        ld.floor_inclination_degrees = np.rad2deg(floor_angle)
        ld.lidar_height = abs(lidar_height)
        self.lidar_proc.publish(ld)

        # debug rospy.logdebugs
        rospy.logdebug("dist_to_pitfall : " + str(dist_to_pitfall))
        rospy.logdebug("dist_to_obstacle : " + str(dist_to_obstacle))
        rospy.logdebug("dist_of_floor : " + str(dist_of_floor))
        rospy.logdebug("floor_angle : " + str(np.rad2deg(floor_angle)))
        rospy.logdebug("lidar_height : " + str(abs(lidar_height)) + "\n")            

        
def polar2cart(rho, phi):
    """
    Helper function - Changing from Polar to Cartesian coordinates
    """
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

    
if __name__ == '__main__':
    rospy.init_node('Lidar_process', log_level=rospy.DEBUG)
    
    myLidarProcess = LidarProcess()
    rospy.spin()
    

