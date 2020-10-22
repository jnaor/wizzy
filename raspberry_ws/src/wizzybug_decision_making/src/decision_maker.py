#!/usr/bin/env python
import sys

sys.path.append('../../../devel/lib/python2.7/dist-packages')

import rospy
import threading

from std_msgs.msg import String
from wizzybug_msgs.msg import *

import smach
import serial

DANGER_TTC = 1.0
WARNING_TTC = 3.0
CLEARANCE_TTC = 5.0
PORT_NAME = "/dev/Relay"
RELAYNUM = "0"  # Support only one relay

"""
Decision making is done based on the recieved messages.
Below classes implements these logics (ttc and flic_button)
"""


class StateMsg():
    def __init__(self):
        # This is due to ttc message being stored in chair_state. Otherwise it wouldn't be required
        self.chair_state = ChairState()

    def process(self):
        return None


class StateTtcMsg(StateMsg):
    def __init__(self, msg=None):
        StateMsg.__init__(self)
        self.chair_state.ttc_msg = msg

    def process(self, prev_state):
        if prev_state is not None and prev_state.state.data == 'WizzyLock':
            return None

        ttc = self.chair_state.ttc_msg.ttc
        if ttc < DANGER_TTC:
            self.chair_state.state.data = 'WizzyA'
        elif DANGER_TTC <= ttc < WARNING_TTC:
            self.chair_state.state.data = 'WizzyB'
        elif WARNING_TTC <= ttc < CLEARANCE_TTC:
            self.chair_state.state.data = 'WizzyC'
        else:
            self.chair_state.state.data = 'WizzyClear'
        return self.chair_state


class StateFlicMsg(StateMsg):
    def __init__(self, msg=None):
        StateMsg.__init__(self)
        self.flic_msg = msg.data

    def process(self,prev_state):
        if self.flic_msg == 'single':
            self.chair_state.state.data = 'WizzyLock'
        elif self.flic_msg == 'double':
            self.chair_state.state.data = 'WizzyClear'
        return self.chair_state


""" Implementation of Waiting Node. 
    Synchronous waiting for subscribed events
"""
class WizzyWaitingState(smach.State):
    def __init__(self, outcomes, latch=False, timeout=None):
        smach.State.__init__(self, outcomes=outcomes, input_keys=['chair_state_in'], output_keys=['chair_state_out'])
        self.msg = None
        self.latch = latch
        self.timeout = timeout
        self.mutex = threading.Lock()
        self.ttc_subscriber = rospy.Subscriber('/ttc', ttc, self.ttc_callback)
        self.flic_subscriber = rospy.Subscriber('/flic_button', String, self.flic_button_callback)

    def ttc_callback(self, msg):
        rospy.logdebug('Executing ttc_callback %f', msg.ttc)
        self.mutex.acquire()
        self.msg = StateTtcMsg(msg)
        self.mutex.release()

    def flic_button_callback(self, msg):
        rospy.logdebug('Executing flic_callback %s', msg.data)
        self.mutex.acquire()
        self.msg = StateFlicMsg(msg)
        self.mutex.release()

    def waitForMsg(self):
        rospy.logdebug('Waiting for message...')
        if self.timeout is not None:
            timeout_time = rospy.Time.now() + rospy.Duration.from_sec(self.timeout)
        while self.timeout is None or rospy.Time.now() < timeout_time:
            self.mutex.acquire()
            if self.msg is not None:
                rospy.logdebug('Got message.')
                message = self.msg
                if not self.latch:
                    self.msg = None
                self.mutex.release()
                return message
            self.mutex.release()
            rospy.sleep(.1)
        rospy.logdebug('Timeout on waiting for message!')
        return None

    """ Main function of the Waiting Note. 
    Recieves a message and executes only if state has changed.
    Special case is first message - in such case we only need a valid new state to execute. 
    """

    def execute(self, userdata):
        state_msg = self.waitForMsg()
        if state_msg is not None:
            prev_state = userdata.chair_state_in
            new_state = state_msg.process(prev_state)
            if new_state is not None and (prev_state is None or (new_state.state.data is not prev_state.state.data)):
                # move to another state
                userdata.chair_state_out = new_state
                return new_state.state.data
            else:
                # keep waiting
                rospy.logdebug('No state change...')
                return 'wait'
        else:
            rospy.logdebug('Aborting...')
            return 'abort'


""" Implmentation of Action Nodes. 
    Controls Relay and Notifies HMI
"""


class WizzyActionState(smach.State):
    def __init__(self, outcomes, relay_state):
        smach.State.__init__(self, outcomes=outcomes, input_keys=['chair_state_in'], output_keys=['chair_state_out'])
        self.relay_state = relay_state
        self.state_publisher = rospy.Publisher('chair_state', ChairState, queue_size=10)
        try:
            self.serial_port = serial.Serial(PORT_NAME, 19200, timeout=1)
        except:
            pass

    def execute(self, userdata):
        # for clarity - action state user data is a chair_state
        new_state = userdata.chair_state_in

        # operate relay
        self.relay_cmd()

        # publish state
        self.state_publisher.publish(new_state)

        # go to waiting state
        userdata.chair_state_out = new_state
        return 'wait'

    def relay_cmd(self):
        str_cmd = "relay " + self.relay_state + " " + RELAYNUM + "\n\r"
        try:
            self.serial_port.write(str.encode(str_cmd))
            rospy.logdebug('Relay CMD:' + str_cmd)
        except:
            rospy.logdebug('Failed to execute relay CMD:' + str_cmd)
            pass


# main
""" Wizzy State Machine is  a star topology statemachine where every Acting Node is connected to the central Waiting Node.
    Waiting Node listens for subscribed events. Once event arrives (ttc,flic,...), a logic is applied to select Action.
    Corresponding Action Node (A,B,C,Clear,Lock) is executed, HMI is notified and graph returns back to Waiting Node.
"""


def main():
    rospy.init_node('decision_maker', log_level=rospy.DEBUG)

    # Create a wizzy state machine
    wizzy_sm = smach.StateMachine(outcomes=['ABORT'])
    wizzy_sm.userdata.prev_state = None
    wizzy_sm.userdata.new_state = None

    # transition map from WAITING state
    waiting_transition_map = {'wait': 'WAITING', 'abort': 'ABORT',
                              'WizzyA': 'WIZZY_A', 'WizzyB': 'WIZZY_B', 'WizzyC': 'WIZZY_C',
                              'WizzyClear': 'WIZZY_CLEAR', 'WizzyLock': 'WIZZY_LOCK'}

    with wizzy_sm:
        smach.StateMachine.add('WAITING',
                               WizzyWaitingState(outcomes=waiting_transition_map.keys(), latch=False, timeout=None),
                               transitions=waiting_transition_map,
                               remapping={'chair_state_in': 'prev_state', 'chair_state_out': 'new_state'})

        smach.StateMachine.add('WIZZY_A',
                               WizzyActionState(outcomes={'wait'}, relay_state="on"),
                               transitions={'wait': 'WAITING'},
                               remapping={'chair_state_in': 'new_state', 'chair_state_out': 'prev_state'})

        smach.StateMachine.add('WIZZY_B',
                               WizzyActionState(outcomes={'wait'}, relay_state="off"),
                               transitions={'wait': 'WAITING'},
                               remapping={'chair_state_in': 'new_state', 'chair_state_out': 'prev_state'})

        smach.StateMachine.add('WIZZY_C',
                               WizzyActionState(outcomes={'wait'}, relay_state="off"),
                               transitions={'wait': 'WAITING'},
                               remapping={'chair_state_in': 'new_state', 'chair_state_out': 'prev_state'})

        smach.StateMachine.add('WIZZY_CLEAR',
                               WizzyActionState(outcomes={'wait'}, relay_state="off"),
                               transitions={'wait': 'WAITING'},
                               remapping={'chair_state_in': 'new_state', 'chair_state_out': 'prev_state'})

        smach.StateMachine.add('WIZZY_LOCK',
                               WizzyActionState(outcomes={'wait'}, relay_state="on"),
                               transitions={'wait': 'WAITING'},
                               remapping={'chair_state_in': 'new_state', 'chair_state_out': 'prev_state'})

    # Execute SMACH plan
    outcome = wizzy_sm.execute()


if __name__ == '__main__':
    main()
