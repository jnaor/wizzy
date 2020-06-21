#!/usr/bin/env python
import can
import os
import struct
import logging
from datetime import datetime
import rospy
from geometry_msgs.msg import Twist

INTERFACE = 'can0'
BITRATE = 105263

# empirically measured linear velocity in m/s
MAX_VELOCITY = 0.5

# empirically measured angular velocity in degrees/s
MAX_ANGULAR_VELOCITY = 40.0

# normalizaton constant (max value of reading)
MAX_VALUE = 100.0

#Rate in HZ
RATE = 10.0

ROS_TOPIC = 'cmd_vel'

last_time_msg_sent = datetime.now()
cmd_pub = None
prev_velocity = 0.0
prev_angular_velocity = 0.0
#Publish topic always or only on value diff
publish_topic_always = True

def sign(x):
    if x<=0:
        return -1
    return 1

def can_recv(iface):
    bus = can.interface.Bus(bustype='socketcan', channel=iface)
    y=0
    x=0 
	
    cmd_vel_msg = Twist()
    rate = rospy.Rate(RATE)
    global cmd_pub	
    for msg in bus:
        if len(msg.data) == 6:
            data_array = struct.unpack('>6b', bytes(msg.data))
            y = data_array[4]
        elif len(msg.data) == 4:
            data_array = struct.unpack('>4b', bytes(msg.data))
            x = data_array[2]

	    if x*y == 0:
           	continue
        
        # normalize according to measured joystick characteristics
        xn, yn = min(abs(x), MAX_VALUE) * sign(x), min(abs(y), MAX_VALUE) * sign(y)

        # forward motion is positive
        velocity = (yn / MAX_VALUE) * MAX_VELOCITY

        # angular velocity. sign flip to make turn around left positive
        angular_velocity = (-xn / MAX_VALUE) * MAX_ANGULAR_VELOCITY

        logging.debug('vel: {}, ang: {}'.format(velocity, angular_velocity))
	dt_now = datetime.now()		
	global last_time_msg_sent;
	diff = dt_now - last_time_msg_sent;
	diff_millisec = diff.microseconds/1000;
	global prev_velocity;
	global prev_angular_velocity;
	global publish_topic_always;
	if last_time_msg_sent == 0 or diff_millisec > RATE*10:
		if prev_velocity != velocity or prev_angular_velocity != angular_velocity or publish_topic_always == True:
			cmd_vel_msg.linear.x = velocity
			cmd_vel_msg.angular.z = angular_velocity
			prev_velocity = velocity; 
			prev_angular_velocity = angular_velocity;
			cmd_pub.publish(cmd_vel_msg)
		last_time_msg_sent = datetime.now()

def disable_iface(iface):
    os.system('sudo ip link set down {}'.format(iface))

def enable_iface(iface, bitrate):
    os.system('sudo /sbin/ip link set {} up type can bitrate {}'.format(iface, bitrate))

def find_bitrate(iface):
#    disable_iface(iface)
#    enable_iface(iface, BITRATE)
    can_recv(iface)

if __name__== "__main__":
    rospy.init_node('can_sniffer', log_level=rospy.DEBUG)
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # set logging level
    logging.basicConfig(level=logging.DEBUG)
	
    # run
    find_bitrate(INTERFACE)
