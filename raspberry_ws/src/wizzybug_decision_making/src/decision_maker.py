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

VALID_MAX_DELAY_SEC_FOR_MSG_TTC = 5.0

PORT_NAME = "/dev/Relay"
RELAYNUM = "0"  # Support only one relay

DISABLE_TIME_CONST = 5.0 # For A State

class DmUtils():
    __q = Queue.Queue(maxsize=1)
    __chair_state = ChairState()
    __serial_port = None
    __state_publisher = None
    __ttc_subscriber = None
    __flic_subscriber = None
 
    def __init__(self):
        if DmUtils.__state_publisher == None:
            DmUtils.__state_publisher = rospy.Publisher('chair_state', ChairState, queue_size=10)
        if DmUtils.__ttc_subscriber == None:
            DmUtils.__ttc_subscriber = rospy.Subscriber('/ttc', ttc, DmUtils.__ttc_callback, queue_size=1)
        if DmUtils.__flic_subscriber == None:
            DmUtils.__flic_subscriber = rospy.Subscriber('/flic_button', String, DmUtils.__flic_callback,queue_size = 1)
        if DmUtils.__serial_port == None:
            try:
                DmUtils.__serial_port = serial.Serial(PORT_NAME, 19200, timeout=1)
            except:
                pass

    @staticmethod
    def __relay(cmd):
        str_cmd = "relay " + cmd + " " + RELAYNUM + "\n\r"
        try:
            DmUtils.__serial_port.write(str.encode(str_cmd))
            rospy.logdebug('Relay CMD:' + str_cmd)
        except:
            rospy.logdebug('Failed to execute relay CMD:' + str_cmd)
        pass
    
    @staticmethod
    def __ttc_callback(msg):
        now = rospy.get_rostime()
        delta_secs = now.to_sec() - msg.header.stamp.to_sec()

        # Ignore old TTC messages
        if delta_secs > VALID_MAX_DELAY_SEC_FOR_MSG_TTC :
            rospy.logdebug('Ignoring TTC message with ttc value of %s. Message is %.3f seconds old, more then %.3f threshold', 
                            msg.ttc, delta_secs, VALID_MAX_DELAY_SEC_FOR_MSG_TTC)
            return

        # TTC message is valid. Continue processing
        rospy.logdebug('Executing ttc_callback %s, sent %.3f seconds ago', 
                        msg.ttc, delta_secs)
        DmUtils.__chair_state.ttc_msg = msg
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
    def __flic_callback(msg):
        rospy.logdebug('Executing flic_callback %s', msg.data)
        if msg.data == 'single':
            DmUtils.post_message('WizzyLock')
        elif msg.data == 'double':
            DmUtils.post_message('WizzyUnlock') 

    @staticmethod
    def publish(state):
        DmUtils.__chair_state.state.data = state
        DmUtils.__state_publisher.publish(DmUtils.__chair_state)

    @staticmethod
    def post_message(message):
        try:
            DmUtils.__q.put(message, block=False)
        except Queue.Full:
            rospy.logdebug("Message Queue is full - skipping %s",message)
    
    @staticmethod
    def wait4message(messages):
        while True:
            msg = DmUtils.__q.get()
            if msg in messages:
                break
        return msg
    
    @staticmethod
    def stop():
        DmUtils.__relay("on")

    @staticmethod
    def release():
        #drain all the messages that might have been accumulated while wizzy stopped
        while DmUtils.__q.empty() == False:
            try:
                message = DmUtils.__q.get_nowait()
                rospy.logdebug("Flushing Message %s",message)
            except Queue.Empty:
                rospy.logdebug("Queue is empty")
                pass
        #release a chair
        DmUtils.__relay("off")


class WizzyClear(smach.State):
    def execute(self, userdata):
        # do state operations
        DmUtils.publish('WizzyClear')
        DmUtils.release()
        
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
        DmUtils.publish('WizzyB')
        
        # Wait for insrtructions
        new_state = DmUtils.wait4message(['WizzyLock','WizzyClear','WizzyA','WizzyC'])
        return new_state

class WizzyC(smach.State):
    def execute(self, userdata):
        # do state operations
        DmUtils.publish('WizzyC')
        
        DmUtils.stop()
        rospy.sleep(DISABLE_TIME_CONST)
        DmUtils.release()

        # Wait for insrtructions
        new_state = DmUtils.wait4message(['WizzyLock','WizzyClear','WizzyA','WizzyB','WizzyC'])
        return new_state

class WizzyLock(smach.State):
    def execute(self, userdata):
        # do state operations
        DmUtils.publish('WizzyLock')
        DmUtils.stop()
        
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
                                WizzyC(outcomes=['WizzyLock','WizzyClear','WizzyA','WizzyB','WizzyC']),
                                {'WizzyLock': 'WIZZY_LOCKED','WizzyClear':'WIZZY_CLEAR','WizzyA':'WIZZY_A','WizzyB':'WIZZY_B','WizzyC':'WIZZY_C'})

        smach.StateMachine.add('WIZZY_LOCKED',WizzyLock(outcomes=['WizzyUnlock']),{'WizzyUnlock': 'WIZZY_CLEAR'})

    # Execute SMACH plan
    outcome = wizzy_sm.execute()

if __name__ == '__main__':
    main()
