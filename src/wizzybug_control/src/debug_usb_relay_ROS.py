#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():

    # rospy.init_node('usb_relay_listener', anonymous=True)
    # rospy.Subscriber("usb_relay_command", String, callback)

    pub = rospy.Publisher('usb_relay_command', String, queue_size=10)
    rospy.init_node('debug_usb_relay_publisher', anonymous=True)
    
    rate = rospy.Rate(0.5) # 0.5hz
    toggle = 1
    while not rospy.is_shutdown():
        if toggle == 1 :
            cmd_str = "on"
            toggle = 0
        else :
            cmd_str = "off"
            toggle = 1
            
        # cmd_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        rospy.loginfo(cmd_str)
        pub.publish(cmd_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass