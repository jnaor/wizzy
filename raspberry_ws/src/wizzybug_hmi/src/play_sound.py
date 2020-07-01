import rospy
from std_msgs.msg import String

from playsound import playsound


def play_bell():
    playsound("./ding_dong.mp3")


def sound_cb(msg):
    print("*** Got" + str(msg))
    if "ButtonSingleClick" in str(msg):
        print("*** Ringing the bell")
        play_bell()


rospy.loginfo("Initializing audio")
rospy.init_node('play_sound', anonymous=True)

# Subscribe
rospy.Subscriber('/wizzy/flic_btn', String, sound_cb)
rospy.spin()
