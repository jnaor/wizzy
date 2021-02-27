#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import tty, termios
import math
import sys, select

WIZZY_MAX_LIN_VEL = 0.5
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
    relay_command = data.data


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('wizzybug_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    relay_subscriber = rospy.Subscriber('usb_relay_command', String, relay_sub_callback, queue_size=1)
    relay_command = 'off'
    target_linear_vel = 0.0
    target_angular_vel = 0.0
    filtered_angular_vel = 0.0
    filtered_linear_vel = 0.0

    rate = rospy.Rate(10)
    key = ''
    twist = Twist()
    alpha = 0.5

    try:
        while not rospy.is_shutdown():
            key = getKey()
            if key == ('w'):
                target_linear_vel = WIZZY_MAX_LIN_VEL
            elif key == ('s'):
                target_linear_vel = -WIZZY_MAX_LIN_VEL
            elif key == ('d'):
                target_angular_vel = -WIZZY_MAX_ANG_VEL
            elif key == ('a'):
                target_angular_vel = WIZZY_MAX_ANG_VEL
            elif key == ('q'):
                target_angular_vel = 0.0
                target_linear_vel = 0.0
                break  # quit node
            else:
                target_angular_vel = 0.0
                target_linear_vel = 0.0

            if relay_command == "on":
                if target_linear_vel > 0:
                    target_linear_vel = 0.0

            filtered_angular_vel = target_angular_vel*(1-alpha) + filtered_angular_vel*alpha
            filtered_linear_vel = target_linear_vel*(1-alpha) + filtered_linear_vel*alpha

            if abs(filtered_linear_vel) < 0.01:
                filtered_linear_vel = 0.0
            if abs(filtered_angular_vel) < 0.01:
                filtered_angular_vel = 0.0

            # print(key)
            # print(target_angular_vel, filtered_angular_vel, target_linear_vel, filtered_linear_vel)

            twist.linear.x = filtered_linear_vel
            twist.angular.z = filtered_angular_vel

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
