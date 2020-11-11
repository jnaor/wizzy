#!/usr/bin/env python

import rospy

from pydub import AudioSegment
from pydub.playback import play

import os

from wizzybug_msgs.msg import ChairState

rospy.loginfo("Initializing audio")
rospy.init_node('play_sound', anonymous=True)

# read camera configuration. default to local if no parameter set by ros
bell_sound_file = rospy.get_param('bell_sound', '../sound/Alert.wav')

# get absolute path
bell_sound_file = os.path.abspath(bell_sound_file)
rospy.loginfo('using bell sound file {}'.format(bell_sound_file))

# read sound file (one time)
bell_sound = AudioSegment.from_wav(bell_sound_file)

def sound_cb(msg):
    """
    Ring the bell only when USB Relay receives an "on" command,
    that stops the Wizzy.
    """
    rospy.loginfo("*** chair state changed to {} : Ringing the bell".format(msg.state.data))
    play(bell_sound)

# Subscribe
rospy.Subscriber("/chair_state", ChairState, sound_cb, queue_size=1)

rospy.spin()
