#!/usr/bin/env python

import time
import threading
import rospy
from led_sections import CommHandler
from std_msgs.msg import *
from wizzybug_msgs.msg import *

class CallbackHandler:

    def __init__(self):
        self.data_pipe = CommHandler(purpose = 'OUTGOING')

    def activation_callback(self, data):
        # Pack and send UDP message:
        #indices = []
        #for obstacle in data.obstacles:
            #idx, dist = self.obstacle_azi_dist(obstacle)
            #if dist < 2 and idx not in indices:
                #indices.append(idx)
        
        idx = self.radians_to_index(data.ttc_msg.ttc_azimuth)

        new_mode = data.state.data

        mode = self.mode_to_char(new_mode)

        self.data_pipe.send_data(mode, idx)
        #self.data_pipe.send_data(mode, indices, len(indices))
        

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

    def mode_to_char(self, char):
        if char == 'wizzy_clear':
            return 'O'
        elif char == 'wizzy_A':
            return 'A'
        elif char == 'wizzy_B':
            return 'B'
        elif char == 'wizzy_C':
            return 'C'
        else:  # error, non existent mode
            return 'e'

    def obstacle_azi_dist(self, obstacle_data):
        obstacle_distance = sqrt(obstacle_data.x**2 + obstacle_data.y**2 + obstacle_data.z**2)
        obstacle_idx = self.radians_to_index(atan2(obstacle_data.y, obstacle_data.x))
        return obstacle_idx, obstacle_distance


if __name__ == "__main__":

    rospy.init_node('wizzy_leds')
    handler = CallbackHandler()
    section_subscriber = rospy.Subscriber('/hmi_commands', ChairState, handler.activation_callback)
    #section_subscriber = rospy.Subscriber('/hmi_commands', Float32, handler.activation_callback)

    try:
        while not rospy.is_shutdown():
            rospy.spin()

    except KeyboardInterrupt: # Quit program cleanly
        for section_index in range(6):
            handler.data_pipe.send_data('wizzy_clear', section_index)
        time.sleep(1)

    except Exception as ex:
        print(ex)


    
