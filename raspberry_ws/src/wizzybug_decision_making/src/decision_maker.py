#!/usr/bin/env python

import rospy
import tf.transformations as transformations
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from wizzybug_msgs.msg import *
import smach
import copy
import smach_ros
import serial

PORT_NAME = "/dev/ttyACM0"
RELAYNUM = "0"  # Support only one relay

DANGER_TTC = 1.0
WARNING_TTC = 3.0
CLEARANCE_TTC = 5.0


class WizzyA(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2', 'outcome3', 'outcome4'])

    def execute(self, userdata):

        if inputs_container.ttc_state != 'outcome1':
            relay_cmd("off")
        if inputs_container.ttc_state != inputs_container.prev_state:
            rospy.logdebug('Executing state A')
        return inputs_container.ttc_state


class WizzyB(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2', 'outcome3', 'outcome4'])

    def execute(self, userdata):

        if inputs_container.ttc_state == 'outcome1':
            relay_cmd("on")
        if inputs_container.ttc_state != inputs_container.prev_state:
            rospy.logdebug('Executing state B')
        return inputs_container.ttc_state


class WizzyC(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2', 'outcome3', 'outcome4'])

    def execute(self, userdata):

        if inputs_container.ttc_state == 'outcome1':
            relay_cmd("on")
        if inputs_container.ttc_state != inputs_container.prev_state:
            rospy.logdebug('Executing state C')
        return inputs_container.ttc_state


class WizzyClear(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2', 'outcome3', 'outcome4'])

    def execute(self, userdata):

        if inputs_container.ttc_state == 'outcome1':
            relay_cmd("on")
        if inputs_container.ttc_state != inputs_container.prev_state:
            rospy.logdebug('Executing state Clear')
        return inputs_container.ttc_state

# class WizzyShutdown(smach.State):


def relay_cmd(relay_cmd_str):
    str_cmd = "relay " + relay_cmd_str + " " + RELAYNUM + "\n\r"
    try:
        inputs_container.serial_port.write(str.encode(str_cmd))
    except:
        pass
    # relevant for sim only
    inputs_container.relay_publisher.publish(relay_cmd_str)

class callback_items:

    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.ttc = 1.0
        self.azimuth = 0
        # self.lidar_data = lidar_data()
        self.ttc_state = 'outcome4'
        self.prev_state = 'outcome4'
        self.chair_state = ChairState()
        self.state_publisher = rospy.Publisher('chair_state', ChairState, queue_size=10)
        # relevant for sim only
        self.relay_publisher = rospy.Publisher('usb_relay_command', String,queue_size=10)
        try:
            self.serial_port = serial.Serial(PORT_NAME, 19200, timeout=1)
        except:
            pass

    def imu_callback(self, data):
        quat = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        self.yaw, self.pitch, self.roll = transformations.euler_from_quaternion(quat, 'rzyx')

    def ttc_callback(self, data):
        self.prev_state = copy.deepcopy(self.ttc_state)
        self.ttc = data.ttc
        self.azimuth = data.ttc_azimuth
        self.chair_state.ttc_msg = data
        if self.ttc < DANGER_TTC:
            self.ttc_state = 'outcome1'
            self.chair_state.state.data = 'WizzyA'
        elif DANGER_TTC < self.ttc < WARNING_TTC:
            self.ttc_state = 'outcome2'
            self.chair_state.state.data = 'WizzyB'
        elif WARNING_TTC < self.ttc < CLEARANCE_TTC:
            self.ttc_state = 'outcome3'
            self.chair_state.state.data = 'WizzyC'
        elif self.ttc > CLEARANCE_TTC:
            self.ttc_state = 'outcome4'
            self.chair_state.state.data= 'WizzyClear'
        self.state_publisher.publish(self.chair_state)

    def lidar_data_callback(self, data):
        self.lidar_data = data

inputs_container = callback_items()

if __name__ == '__main__':

    rospy.init_node('decision_maker', log_level = rospy.DEBUG)
    relay_cmd("off")
    # Subscribers

    imu_subscriber = rospy.Subscriber('/imu/data', Imu, inputs_container.imu_callback)
    ttc_subscriber = rospy.Subscriber('/ttc', ttc, inputs_container.ttc_callback)
    # lidar_data_subscriber = rospy.Subscriber('/myLidar/lidar_proc', lidar_data, inputs_container.lidar_data_callback)



    # State machine
    wizzy_sm = smach.StateMachine(outcomes=['outcome5'])

    with wizzy_sm:
        smach.StateMachine.add('WizzyA', WizzyA(), transitions={'outcome1' : 'WizzyA','outcome2' : 'WizzyB',
                                                                'outcome3' : 'WizzyC', 'outcome4': 'WizzyClear'})
        smach.StateMachine.add('WizzyB', WizzyB(), transitions={'outcome1' : 'WizzyA','outcome2' : 'WizzyB',
                                                                'outcome3' : 'WizzyC', 'outcome4': 'WizzyClear'})
        smach.StateMachine.add('WizzyC', WizzyC(), transitions={'outcome1' : 'WizzyA','outcome2' : 'WizzyB',
                                                                'outcome3' : 'WizzyC', 'outcome4': 'WizzyClear'})
        smach.StateMachine.add('WizzyClear', WizzyClear(), transitions={'outcome1' : 'WizzyA','outcome2' : 'WizzyB',
                                                                'outcome3' : 'WizzyC', 'outcome4': 'WizzyClear'})

    outcome = wizzy_sm.execute()
