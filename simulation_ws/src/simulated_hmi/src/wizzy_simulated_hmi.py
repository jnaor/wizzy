#!/usr/bin/env python

import time
import threading
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3
from led_sections import LedSection
from vibration_motors import VibrationMotor
from std_msgs.msg import *
from wizzybug_msgs.msg import *
from math import *

class CallbackHandler:

    def __init__(self, motors, leds):
        self.motors = motors
        self.leds = leds

    def activation_callback(self, data):        
        direction_type = 2
        if direction_type == 1:  # A placeholder in case position-based vibration is needed:           
            indices = []
            for obstacle in data.obstacles:
                idx, dist = self.obstacle_azi_dist(obstacle)
                if dist < 2 and idx not in indices:
                    indices.append(idx)
        
        else:
            indices = [self.radians_to_index(data.ttc_msg.ttc_azimuth)]

        # Check for active motors:
        for idx in indices:
            if self.motors[idx].is_active:
                self.motors[idx].turn_off()
            if self.motors[idx].is_active:
                self.motors[idx].turn_off()
        
        # Making absolute sure that they all turned off:
        time.sleep(VibrationMotor.DT*2)

        # Apply new data:
        for idx in indices:
            new_mode = data.state.data

            self.motors[idx].set_mode(new_mode)
            self.motors[idx].begin_sequence()

            self.leds[idx].set_mode(new_mode)
            self.leds[idx].begin_sequence()

            print('motor node',idx, new_mode, self.motors[idx].is_active)
        

    def radians_to_index(self, angle):  # Motors will be different than LED!
        # Quick and dirty, there can be a more efficient way:
        if -0.5236 <= angle <= 0.5236:
            return 0
        elif 0.5236 <= angle <= 1.5708:
            return 1
        elif 1.5708 <= angle <= 2.618:
            return 2
        elif -2.618 <= angle <= -1.5708:
            return 4
        elif -1.5708 <= angle <= -0.5236:
            return 5
        else:  # Only remaining interval is facing backwards
            return 3

    def mode_to_char(self, mode):
        if mode == 'wizzy_clear':
            return 'O'
        elif mode == 'wizzy_A':
            return 'A'
        elif mode == 'wizzy_B':
            return 'B'
        elif mode == 'wizzy_C':
            return 'C'
        else:  # error, non existent mode
            return 'e'

    def obstacle_azi_dist(self, obstacle_data):
        obstacle_distance = sqrt(obstacle_data.x**2 + obstacle_data.y**2 + obstacle_data.z**2)
        obstacle_idx = self.radians_to_index(atan2(obstacle_data.y, obstacle_data.x))
        return obstacle_idx, obstacle_distance


if __name__ == "__main__":

    rospy.init_node('simulated_hmi')

    delta_time = 0.1

    hmi_height = 2  # Meters
    led_radius = 2
    motor_radius = 3

    motor_markers = MarkerArray()
    led_markers = MarkerArray()

    for direction_idx in range(6):
        current_marker = Marker()
        current_marker.header.frame_id = 'map'
        current_marker.type = Marker.CUBE
        current_marker.color = ColorRGBA(1, 1, 1, 0.1)
        current_marker.scale = Vector3(0.5, 0.5, 0.5)
        current_marker.ns = 'motor_markers'
        current_marker.id = direction_idx
        angle = direction_idx*3.14*2/6
        current_marker.pose.position.x = motor_radius*cos(angle)
        current_marker.pose.position.y = motor_radius*sin(angle)
        current_marker.pose.position.z = hmi_height
        motor_markers.markers.append(current_marker)

    for direction_idx in range(6):
        for led_idx in range(4):
            current_marker = Marker()
            current_marker.header.frame_id = 'map'
            current_marker.type = Marker.SPHERE
            current_marker.color.a = 0.1
            current_marker.scale = Vector3(0.5, 0.5, 0.5)
            current_marker.ns = 'led_markers'
            current_marker.id = led_idx + direction_idx*4
            angle = (led_idx + direction_idx*4 - 2)*2*3.14/24
            current_marker.pose.position.x = led_radius*cos(angle)
            current_marker.pose.position.y = led_radius*sin(angle)
            current_marker.pose.position.z = hmi_height
            led_markers.markers.append(current_marker)
    
    motor_list = [VibrationMotor(0, motor_markers.markers),
                  VibrationMotor(1, motor_markers.markers),
                  VibrationMotor(2, motor_markers.markers),
                  VibrationMotor(3, motor_markers.markers),
                  VibrationMotor(4, motor_markers.markers),
                  VibrationMotor(5, motor_markers.markers)]
    motor_threads = [threading.Thread(target = mot.loop_sequence) for mot in motor_list]
    for current_thread in motor_threads:
        current_thread.daemon=True
        current_thread.start()

    section_list = [LedSection(0, led_markers.markers), 
                    LedSection(1, led_markers.markers), 
                    LedSection(2, led_markers.markers), 
                    LedSection(3, led_markers.markers), 
                    LedSection(4, led_markers.markers), 
                    LedSection(5, led_markers.markers)]      
    section_threads = [threading.Thread(target = sec.loop_sequence) for sec in section_list]
    for current_thread in section_threads:
        current_thread.daemon=True
        current_thread.start() 

    handler = CallbackHandler(motor_list, section_list)
    section_subscriber = rospy.Subscriber('/hmi_commands', ChairState, handler.activation_callback)
    motors_publisher = rospy.Publisher('hmi_motors', MarkerArray, queue_size = 10)
    leds_publisher = rospy.Publisher('hmi_leds', MarkerArray, queue_size = 10)

    try:
        while not rospy.is_shutdown():
            time.sleep(delta_time)
            # Update timestamps before publishing:
            current_timestamp = rospy.Time.now()
            for marker_idx in range(24):
                led_markers.markers[marker_idx].header.stamp = current_timestamp
                if marker_idx < 6:
                    motor_markers.markers[marker_idx].header.stamp = current_timestamp

            motors_publisher.publish(motor_markers)
            leds_publisher.publish(led_markers)

    except KeyboardInterrupt: # Quit program cleanly
        time.sleep(1)

    except Exception as ex:
        print(ex)


    
