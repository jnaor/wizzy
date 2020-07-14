#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import socket

HOST = "127.0.0.1"
PORT = 6666

# a workaround for pulseaudio sudo-less audio playing woes
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

rospy.loginfo("Initializing audio")
rospy.init_node('play_sound', anonymous=True)

# read camera configuration. default to local if no parameter set by ros
bell_sound_file = rospy.get_param('bell_sound', '../sound/ding_dong.mp3')
rospy.loginfo('using bell sound file {}'.format(bell_sound_file))

def play_bell():
    sock.sendall(b'ding_dong')

def sound_cb(msg):
    """
    Ring the bell only when USB Relay receives an "on" command,
    that stops the Wizzy.
    """    
    rospy.loginfo('play_sound received message {}'.format(msg))    
    if "on" in str(msg):
        rospy.loginfo("*** Got USB Relay ON : Ringing the bell")
        play_bell()

# Subscribe
rospy.Subscriber("usb_relay_command", String, sound_cb)

rospy.spin()