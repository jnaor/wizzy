#!/usr/bin/env python

import rospy
from std_msgs.msg import String

# TODO: use direct method
import subprocess

# currently does not work
# from playsound import playsound

rospy.loginfo("Initializing audio")
rospy.init_node('play_sound', anonymous=True)

# read camera configuration. default to local if no parameter set by ros
bell_sound_file = rospy.get_param('bell_sound', '../sound/ding_dong.mp3')
rospy.loginfo('using bell sound file {}'.format(bell_sound_file))


def play_bell():
    subprocess.Popen(['cvlc', bell_sound_file])
    # playsound(bell_sound_file)


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
#rospy.Subscriber('/wizzy/flic_btn', String, sound_cb)
rospy.Subscriber("usb_relay_command", String, sound_cb)

rospy.spin()
