#!/usr/bin/env python

import time
import threading
import rospy
from led_sections import CommHandler
from std_msgs.msg import *
from wizzybug_msgs.msg import *


def index_to_mode(idx):
    modded_idx = idx % 3
    print(idx, modded_idx)
    if modded_idx == 0:
        return 'wizzy_A'
    elif modded_idx == 1:
        return 'wizzy_B'
    elif modded_idx == 2:
        return 'wizzy_C'
    else:
        return 'wizzy_clear'


if __name__ == "__main__":

    rospy.init_node('hmi_tester')
    chair_state_pub = rospy.Publisher('/chair_state', ChairState, queue_size=10)
    chair_state = ChairState()
    rate = rospy.Rate(0.15)
    count = -2    
    time.sleep(1)

    while not rospy.is_shutdown():
        chair_state.ttc_msg.ttc_azimuth = count * 1.047
        chair_state.state.data = index_to_mode(count)
        chair_state_pub.publish(chair_state)
        print(chair_state.ttc_msg.ttc_azimuth, chair_state.state.data)
        count += 1
        if count == 4:
            count = -2
        rate.sleep()




    
