import can
import os
import math
import struct
import logging
import rospy
from geometry_msgs.msg import Twist

INTERFACE = 'can0'
BITRATE = 105263

# empirically measured linear velocity in m/s
MAX_VELOCITY = 0.5

# empirically measured angular velocity in rad/s
MAX_ANGULAR_VELOCITY = math.radians(40)

# normalizaton constant (max value of reading)
MAX_VALUE = 100.0

def sign(x):
    if x<=0:
        return -1
    return 1

def can_recv(iface):
    bus = can.interface.Bus(bustype='socketcan', channel=iface)
    y=0
    x=0 
    msg = bus[-1]
    if len(msg.data) == 6:
        data_array = struct.unpack('>6b', bytes(msg.data))
        y = data_array[4]
    elif len(msg.data) == 4:
        data_array = struct.unpack('>4b', bytes(msg.data))
        x = data_array[2]

    # normalize according to measured joystick characteristics
    xn, yn = min(abs(x), MAX_VALUE) * sign(x), min(abs(y), MAX_VALUE) * sign(y)

    # forward motion is positive
    velocity = (yn / MAX_VALUE) * MAX_VELOCITY

    # angular velocity. sign flip to make turn around left positive
    angular_velocity = (-xn / MAX_VALUE) * MAX_ANGULAR_VELOCITY
    rospy.logdebug('vel: {}, ang: {}'.format(velocity, angular_velocity))

    return velocity, angular_velocity

def disable_iface(iface):
    os.system('sudo ip link set down {}'.format(iface))

def enable_iface(iface, bitrate):
    os.system('sudo /sbin/ip link set {} up type can bitrate {}'.format(iface, bitrate))

def find_bitrate(iface):
    disable_iface(iface)
    enable_iface(iface, BITRATE)
    v,w = can_recv(iface)
    return v,w

if __name__== "__main__":
    rospy.init_node('can_sniffer', log_level=rospy.DEBUG)
    #
    disable_iface(INTERFACE)
    enable_iface(INTERFACE, BITRATE)
    #
    cmd_vel_msg = Twist()
    rate = rospy.Rate(10)
    #
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # run
    while not rospy.is_shutdown():
        v,w = can_recv(INTERFACE)
        cmd_vel_msg.linear.x = v
        cmd_vel_msg.angular.z = w
        cmd_pub.publish(cmd_vel_msg)


