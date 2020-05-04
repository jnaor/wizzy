#!/usr/bin/env python

import time
import rospy
import threading
from vibration_motors import VibrationMotor
import pigpio
from std_msgs.msg import *
from wizzybug_msgs.msg import *
from math import sqrt, atan2

class CallbackHandler:

    MIN_DIST = 1.0

    def __init__(self, motors, threads):
        self.motors = motors
        self.threads = threads

    def activation_callback(self, data):
        #indices = []
        #for obstacle in data.obstacles:
            #idx, dist = self.obstacle_azi_dist(obstacle)
            #if dist < 2 and idx not in indices:
                #indices.append(idx)

        idx = self.radians_to_index(data.ttc_msg.ttc_azimuth)

        # Check for active motors:
        if self.motors[idx].is_active:
            self.motors[idx].turn_off()
        
        # Making absolute sure that they all turned off:
        time.sleep(VibrationMotor.DT*2)

        # Apply new data:
        new_mode = data.state.data
        self.motors[idx].set_mode(new_mode)
        self.motors[idx].begin_sequence()

        print(idx, new_mode, self.motors[idx].is_active)


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

    def obstacle_azi_dist(self, obstacle_data):
        obstacle_distance = sqrt(obstacle_data.x**2 + obstacle_data.y**2 + obstacle_data.z**2)
        obstacle_idx = self.radians_to_index(atan2(obstacle_data.y, obstacle_data.x))
        return obstacle_idx, obstacle_distance


if __name__ == "__main__":

    rospy.init_node('wizzy_motors')
    gpio = pigpio.pi()

    delta_time = 0.01
    
    
    motor_list = [VibrationMotor(0, 26, gpio),
                  VibrationMotor(1, 26, gpio),
                  VibrationMotor(2, 26, gpio),
                  VibrationMotor(3, 26, gpio),
                  VibrationMotor(4, 26, gpio),
                  VibrationMotor(5, 26, gpio)]

    motor_threads = [threading.Thread(target = mot.loop_sequence) for mot in motor_list]
    for current_thread in motor_threads:
        current_thread.daemon=True
        current_thread.start()
    
    handler = CallbackHandler(motor_list, motor_threads)

    motor_subscriber = rospy.Subscriber('/hmi_commands', ChairState, handler.activation_callback)
    #motor_subscriber = rospy.Subscriber('/hmi_commands', Float32, handler.activation_callback)

    try:        
        while not rospy.is_shutdown():
            time.sleep(delta_time)

    except KeyboardInterrupt: # Quit program cleanly
        for mot in motor_list:
            mot.turn_off()       
        time.sleep(1)
    
    except Exception as ex:
        print(ex)

    
