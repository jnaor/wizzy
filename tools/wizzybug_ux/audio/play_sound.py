# import os


import sys
sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
import rospy
from std_msgs.msg import String

from playsound import playsound

# def play_wav(wav):
	# if wav.endswith('.wav'):
		# os.system('aplay ' + wav)

# def play_B5():
	# play_wav(os.getcwd()+'/B5.wav')


# def play_Alert():
        # play_wav(os.getcwd()+'/Alert_C1_Trimmed.wav')

def play_bell() :
    playsound("./ding_dong.mp3")
        
def sound_cb(msg): 
    print ("*** Got" + str(msg))            
    if "ButtonSingleClick" in str(msg) :
        print ("*** Ringing the bell")
        play_bell()
        
print ("Initing sound")        
rospy.init_node('play_sound', anonymous=True)
# Subscribe 
rospy.Subscriber('/wizzy/flic_btn', String, sound_cb)
rospy.spin()
