#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import tty, termios
import math
import sys, select

WIZZY_MAX_LINE_VEL = 0.5
WIZZY_LIN_STEP = 0.1
WIZZY_MAX_ANG_VEL = 1.57 / 2
WIZZY_ANG_STEP = 0.2
relay_command = 'off'

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def relay_sub_callback(data):
    global relay_command
    relay_command =data.data

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('wizzybug_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    relay_command = rospy.Subscriber('usb_relay_command', String, relay_sub_callback, queue_size=1)
    target_linear_vel = 0.0
    target_angular_vel = 0.0
    rate = rospy.Rate(10)
    key = ''
    try:
        while not rospy.is_shutdown():
            key = getKey()
            if key == ('w'):
                target_linear_vel += WIZZY_LIN_STEP
            elif key == ('s'):
                target_linear_vel -= WIZZY_LIN_STEP
            elif key == ('d'):
                target_angular_vel -= WIZZY_ANG_STEP
            elif key == ('a'):
                target_angular_vel += WIZZY_ANG_STEP
            elif key == ('p'):
                target_angular_vel = 0.0
                target_linear_vel = 0.0
            elif key == ('q'):
                target_angular_vel = 0.0
                target_linear_vel = 0.0
                break
            if relay_command == "on":
                if target_linear_vel > 0:
                    target_linear_vel = 0.0
                    target_linear_vel = 0.0
            # else:
            #     target_angular_vel = 0.0
            #     target_linear_vel = 0.0


            twist = Twist()
            if abs(target_linear_vel) >= WIZZY_MAX_LINE_VEL:
                target_linear_vel = math.copysign(WIZZY_MAX_LINE_VEL, target_linear_vel)
            if abs(target_angular_vel) >= WIZZY_MAX_ANG_VEL:
                target_angular_vel = math.copysign(WIZZY_MAX_LINE_VEL, target_angular_vel)

            twist.linear.x = target_linear_vel
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = target_angular_vel

            pub.publish(twist)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
