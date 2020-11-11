#!/usr/bin/env python
import sys

sys.path.append('../../../devel/lib/python2.7/dist-packages')

import rospy
from std_msgs.msg import String
from wizzybug_msgs.msg import *

import smach
import serial

import Queue
import time


DANGER_TTC = 2.0
WARNING_TTC = 4.0
CLEARANCE_TTC = 6.0

PORT_NAME = "/dev/Relay"
RELAYNUM = "0"  # Support only one relay

DISABLE_TIME_CONST = 5.0 # For A State

class DmUtils():
    q = Queue.Queue(maxsize=0)
    chair_state = ChairState()
    serial_port = None
    state_publisher = None
    ttc_subscriber = None
    flic_subscriber = None
    
    def __init__(self):
        if DmUtils.state_publisher == None:
            DmUtils.state_publisher = rospy.Publisher('chair_state', ChairState, queue_size=10)
        if DmUtils.ttc_subscriber == None:
            DmUtils.ttc_subscriber = rospy.Subscriber('/ttc', ttc, DmUtils.ttc_callback)
        if DmUtils.flic_subscriber == None:
            DmUtils.flic_subscriber = rospy.Subscriber('/flic_button', String, DmUtils.flic_callback)
        if DmUtils.serial_port == None:
            try:
                DmUtils.serial_port = serial.Serial(PORT_NAME, 19200, timeout=1)
            except:
                pass

    @staticmethod
    def publish(state):
        DmUtils.chair_state.state.data = state
        DmUtils.state_publisher.publish(DmUtils.chair_state)

    @staticmethod
    def relay(cmd):
        str_cmd = "relay " + cmd + " " + RELAYNUM + "\n\r"
        try:
            DmUtils.serial_port.write(str.encode(str_cmd))
            rospy.logdebug('Relay CMD:' + str_cmd)
        except:
            rospy.logdebug('Failed to execute relay CMD:' + str_cmd)
        pass
    
    @staticmethod
    def ttc_callback(msg):
        rospy.logdebug('Executing ttc_callback %s', msg.ttc)
        DmUtils.chair_state.ttc_msg = msg
        ttc = msg.ttc
        
        # TODO: Add logic according to obstacale classification
        if ttc < DANGER_TTC:
            DmUtils.post_message('WizzyC')
        elif DANGER_TTC <= ttc < WARNING_TTC:
            DmUtils.post_message('WizzyB')
        elif WARNING_TTC <= ttc < CLEARANCE_TTC:
            DmUtils.post_message('WizzyA')
        else:
            DmUtils.post_message('WizzyClear')
    
    @staticmethod
    def flic_callback(msg):
        rospy.logdebug('Executing flic_callback %s', msg.data)
        if msg.data == 'single':
            DmUtils.post_message('WizzyLock')
        elif msg.data == 'double':
            DmUtils.post_message('WizzyUnlock') 

    @staticmethod
    def post_message(message):
        DmUtils.q.put(message)
    
    @staticmethod
    def wait4message(messages):
        while True:
            msg = DmUtils.q.get()
            if msg in messages:
                break
        return msg

class WizzyClear(smach.State):
    def execute(self, userdata):
        # do state operations
        DmUtils.publish('WizzyClear')
        DmUtils.relay("off")
        
        # Wait for insrtructions
        new_state = DmUtils.wait4message(['WizzyLock','WizzyA','WizzyB','WizzyC'])
        return new_state

class WizzyA(smach.State):
    def execute(self, userdata):
        # do state operations
        DmUtils.publish('WizzyA')
        
        # Wait for insrtructions
        new_state = DmUtils.wait4message(['WizzyLock','WizzyClear','WizzyB','WizzyC'])
        return new_state

class WizzyB(smach.State):
    def execute(self, userdata):
        # do state operations
        DmUtils.state_publisher.publish('WizzyB')
        
        # Wait for insrtructions
        new_state = DmUtils.wait4message(['WizzyLock','WizzyClear','WizzyA','WizzyC'])
        return new_state

class WizzyC(smach.State):
    def execute(self, userdata):
        # do state operations
        DmUtils.publish('WizzyC')
        
        DmUtils.relay("on")
        time.sleep(DISABLE_TIME_CONST)
        DmUtils.relay("off")

        # Wait for insrtructions
        new_state = DmUtils.wait4message(['WizzyLock','WizzyClear','WizzyA','WizzyB'])
        return new_state

class WizzyLock(smach.State):
    def execute(self, userdata):
        # do state operations
        DmUtils.publish('WizzyLock')
        DmUtils.relay("on")
        
        # Wait for insrtructions
        new_state = DmUtils.wait4message(['WizzyUnlock'])

        return new_state

# main
def main():
    rospy.init_node('decision_maker', log_level=rospy.DEBUG)
    
    # Initialize all the utilities
    DmUtils()

    # Create a wizzy state machine
    wizzy_sm = smach.StateMachine(outcomes=['ABORT'])
    
    with wizzy_sm:
        smach.StateMachine.add('WIZZY_CLEAR',
                                WizzyClear(outcomes=['WizzyLock','WizzyA','WizzyB','WizzyC']),
                                {'WizzyLock': 'WIZZY_LOCKED','WizzyA':'WIZZY_A','WizzyB':'WIZZY_B','WizzyC':'WIZZY_C'})
        
        smach.StateMachine.add('WIZZY_A',
                                WizzyA(outcomes=['WizzyLock','WizzyClear','WizzyB','WizzyC']),
                                {'WizzyLock': 'WIZZY_LOCKED','WizzyClear':'WIZZY_CLEAR','WizzyB':'WIZZY_B','WizzyC':'WIZZY_C'})

        smach.StateMachine.add('WIZZY_B',
                                WizzyB(outcomes=['WizzyLock','WizzyClear','WizzyA','WizzyC']),
                                {'WizzyLock': 'WIZZY_LOCKED','WizzyClear':'WIZZY_CLEAR','WizzyA':'WIZZY_A','WizzyC':'WIZZY_C'})

        smach.StateMachine.add('WIZZY_C',
                                WizzyC(outcomes=['WizzyLock','WizzyClear','WizzyA','WizzyB']),
                                {'WizzyLock': 'WIZZY_LOCKED','WizzyClear':'WIZZY_CLEAR','WizzyA':'WIZZY_A','WizzyB':'WIZZY_B'})

        smach.StateMachine.add('WIZZY_LOCKED',WizzyLock(outcomes=['WizzyUnlock']),{'WizzyUnlock': 'WIZZY_CLEAR'})

    # Execute SMACH plan
    outcome = wizzy_sm.execute()

if __name__ == '__main__':
    main()
