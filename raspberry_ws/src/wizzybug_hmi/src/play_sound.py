#!/usr/bin/env python

import rospy
from std_msgs.msg import String

from pydub import AudioSegment
from pydub.playback import play

import os

rospy.loginfo("Initializing audio")
rospy.init_node('play_sound', anonymous=True)

# read camera configuration. default to local if no parameter set by ros
bell_sound_file = rospy.get_param('bell_sound', 'sound/B5.wav')

# get absolute path
bell_sound_file = os.path.abspath(bell_sound_file)
rospy.loginfo('using bell sound file {}'.format(bell_sound_file))

# read sound file (one time)
bell_sound = AudioSegment.from_file(bell_sound_file)

def sound_cb(msg):
    """
    Ring the bell only when USB Relay receives an "on" command,
    that stops the Wizzy.
    """    
    rospy.loginfo('play_sound received message {}'.format(msg))    
    if "on" in str(msg):
        rospy.loginfo("*** Got USB Relay ON : Ringing the bell")
        play(bell_sound)

# Subscribe
rospy.Subscriber("usb_relay_command", String, sound_cb)

rospy.spin()
