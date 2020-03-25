#!/usr/bin/env python
import rospy

from std_msgs.msg       import Float64

# Input measurements from the RPLidar driver node
from sensor_msgs.msg    import LaserScan

# Output measurements of hazards to the DM node
from wizzybug_msgs.msg  import lidar_data

# Used to run the Audio node. Replaced by code in DM node.
# import subprocess

import numpy as np


class LidarProcess :
    """
    Process lidar sensor input from topic /scan, 
    and publish obstacle distance and LidarProcess state (i.e., obstacle type)
    """
    def __init__ (self) :
        # Publish - TODO - Aggregate these four variables to a single message
        # self.dist_to_pitfall_pub   = rospy.Publisher ('/myLidar/dist_to_pitfall',   Float64, queue_size=10)
        # self.dist_to_obstacle_pub  = rospy.Publisher ('/myLidar/dist_to_obstacle',  Float64, queue_size=10)
        # self.dist_of_floor_pub     = rospy.Publisher ('/myLidar/dist_of_floor',     Float64, queue_size=10)
        # self.state_pub             = rospy.Publisher ('/myLidar/state',             String, queue_size=10)
        self.lidar_proc             = rospy.Publisher ('/myLidar/lidar_proc',       lidar_data, queue_size=10)       
        
        # Subscribe 
        rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        
        # Debug publish        
        self.floor_angle_pub       = rospy.Publisher ('/myLidar/floor_angle',        Float64, queue_size=10)
        self.lidar_height_pub      = rospy.Publisher ('/myLidar/lidar_height',        Float64, queue_size=10)
        self.myScan_pub            = rospy.Publisher ('/myLidar/myScan', LaserScan,  queue_size=10)
                
        # Lidar detection state
        self.prev_state = "OK"  # Initialize previous state for debugging

        self.start_time = 0
        
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
        
        # Initialize State
        state = "OK"

        # Hazards limits
        MIN_ALLOWED_OBSTACLE_DIST = (1.2) # One meter minimum for obstacles, otherwise activate relay
        MIN_ALLOWED_PITFALL_DIST  = (0.6) # One meter minimum for pitfall, otherwise activate relay
        MIN_ALLOWED_FLOOR_DIST    = (0.4) # 60cm minimum of floor, otherwise activate relay

        
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

        # Debugging - Pretty print floats
        # TODO: delete this
        np.set_printoptions(suppress=True, precision=2)

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
        self.lidar_proc.publish(ld)
        
        # self.dist_to_pitfall_pub.publish(dist_to_pitfall) 
        # self.dist_to_obstacle_pub.publish(dist_to_obstacle)  
        # self.dist_of_floor_pub.publish(dist_of_floor)
         
        # Debug publish 
        self.floor_angle_pub.publish(np.rad2deg(floor_angle))
        self.lidar_height_pub.publish(abs(lidar_height))         
        # Monitor the measurement window for filtering
        self.myScan_pub.publish(msg)

        # Replace the DM logic
        if dist_to_pitfall <= MIN_ALLOWED_PITFALL_DIST :
            state = "DIST_TO_PITFALL_ERR"
        elif dist_to_obstacle <= MIN_ALLOWED_OBSTACLE_DIST :
            state = "DIST_TO_OBSTACLE_ERR"
        elif dist_of_floor <= MIN_ALLOWED_FLOOR_DIST :
            state = "DIST_OF_FLOOR_ERR"

        # debug prints
        if state != "OK" :
            curr_time = rospy.get_time()
            print ("State : " + state)
            print ("dist_to_pitfall : " + str(dist_to_pitfall))
            print ("dist_to_obstacle : " + str(dist_to_obstacle))
            print ("dist_of_floor : " + str(dist_of_floor))
            print ("floor_angle : " + str(np.rad2deg(floor_angle)))
            print ("lidar_height : " + str(abs(lidar_height)) + "\n")

            # if self.start_time == 0 or (curr_time - self.start_time) > 5.0 :
                # self.start_time = rospy.get_time()
                # subprocess.run(["sh", "../../wizzybug_ux/audio/playAlertC1.sh"])
            
        # self.state_pub.publish(state)                

        
def polar2cart(rho, phi):
    """
    Helper function - Changing from Polar to Cartesian coordinates
    """
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

    
if __name__ == '__main__':
    rospy.init_node('Lidar_process')
    myLidarProcess = LidarProcess()
    rospy.spin()
    

