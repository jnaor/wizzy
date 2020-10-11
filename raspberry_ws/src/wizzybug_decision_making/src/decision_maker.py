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

PORT_NAME = "/dev/Relay"
RELAYNUM = "0"  # Support only one relay

DANGER_TTC = 1.0
WARNING_TTC = 3.0
CLEARANCE_TTC = 5.0
STATE_HOLD = 0.05
ON_HOLD = 0.1


""" base class for wizzy states """
class WizzyState(smach.State):
    def __init__(self, outcomes):
        # init state
        smach.State.__init__(self, outcomes=outcomes)

    # do this when entering state
    def execute(self, userdata):

        rospy.logdebug('relay command {}'.format(userdata))

        # TODO: misuse of userdata            
        relay_cmd(userdata)

        # sleep between state checks
        rospy.sleep(ON_HOLD)

        # return state from global structure
        return inputs_container.state

class WizzyA(WizzyState):
    
    def execute(self, userdata):
        rospy.logdebug('Entering WizzyA state')
        return super(WizzyA, self).execute("on")


class WizzyB(WizzyState):
    
    def execute(self, userdata):
        rospy.logdebug('Entering WizzyB state')
        return super(WizzyB, self).execute("off")


class WizzyC(WizzyState):

    def execute(self, userdata):
        rospy.logdebug('Entering WizzyC state')
        return super(WizzyC, self).execute("off")


class WizzyClear(WizzyState):
    def execute(self, userdata):
        rospy.logdebug('Entering WizzyClear state')
        return super(WizzyClear, self).execute("off")

class WizzyLock(WizzyState):
   def execute(self, userdata):

        rospy.logdebug('Received external lock message')
        return super(WizzyLock, self).execute("on")        


def relay_cmd(relay_cmd_str):
    if inputs_container.relay_state != relay_cmd_str:
        str_cmd = "relay " + relay_cmd_str + " " + RELAYNUM + "\n\r"
        try:
            inputs_container.serial_port.write(str.encode(str_cmd))
            rospy.loginfo('Relay CMD:' + str_cmd)
        except:
            pass
        # relevant for sim only
        # inputs_container.relay_publisher.publish(relay_cmd_str)
        inputs_container.relay_state = relay_cmd_str

class callback_items:

    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.ttc = 1.0
        self.azimuth = 0
        # self.lidar_data = lidar_data()
        self.state = 'ttc_clear'
        self.prev_state = 'ttc_clear'

        self.chair_state = ChairState()
        self.relay_state = "off"
        self.state_publisher = rospy.Publisher('chair_state', ChairState, queue_size=10)
        
        # relevant for sim only
        # dead code?
        # self.relay_publisher = rospy.Publisher('usb_relay_command', String,queue_size=10)
        
        try:
            self.serial_port = serial.Serial(PORT_NAME, 19200, timeout=1)
        except:
            pass

    def imu_callback(self, data):
        quat = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        self.yaw, self.pitch, self.roll = transformations.euler_from_quaternion(quat, 'rzyx')

    def ttc_callback(self, data):
        # remember current state
        self.prev_state = self.state

        self.ttc = data.ttc
        self.azimuth = data.ttc_azimuth
        self.chair_state.ttc_msg = data
        if self.ttc < DANGER_TTC:
            self.state = 'ttc_danger'
            self.chair_state.state.data = 'WizzyA'
        elif DANGER_TTC < self.ttc < WARNING_TTC:
            self.state = 'ttc_warning'
            self.chair_state.state.data = 'WizzyB'
        elif WARNING_TTC < self.ttc < CLEARANCE_TTC:
            self.state = 'ttc_notification'
            self.chair_state.state.data = 'WizzyC'
        elif self.ttc > CLEARANCE_TTC:
            self.state = 'ttc_clear'
            self.chair_state.state.data= 'WizzyClear'
        if self.prev_state != self.state:
            self.state_publisher.publish(self.chair_state)
        #rospy.sleep(STATE_HOLD)

    def flic_button_callback(self, msg):
        print('flic_button_callback data: {}'.format(msg.data))
        
        # remember
        self.prev_state = self.state

        if msg.data == 'single':
            self.chair_state.state.data = 'WizzyLock'
            self.state = 'caretaker_lock'

        elif msg.data == 'double':
            self.chair_state.state.data = 'WizzyClear'
            self.state = 'ttc_clear'

        # no action for "hold" operation for now

        # publish to notify HMI 
        if self.prev_state != self.state:

            # publish
            self.state_publisher.publish(self.chair_state)


inputs_container = callback_items()

if __name__ == '__main__':

    rospy.init_node('decision_maker', log_level = rospy.INFO)
    relay_cmd("off")
    # Subscribers

    # IMU inactive for now
    # imu_subscriber = rospy.Subscriber('/imu/data', Imu, inputs_container.imu_callback)
    ttc_subscriber = rospy.Subscriber('/ttc', ttc, inputs_container.ttc_callback)

    # subscribe to flic button messages
    flic_subscriber = rospy.Subscriber('/flic_button', String, inputs_container.flic_button_callback)

    smach.set_loggers(rospy.logdebug, rospy.logwarn, rospy.logdebug, rospy.logerr)

    # State machine
    wizzy_sm = smach.StateMachine(outcomes=['outcome5'])

    # transition map for all states
    transition_map = {'ttc_danger' : 'WizzyA', 'ttc_warning' : 'WizzyB', 
                'ttc_notification' : 'WizzyC', 'ttc_clear': 'WizzyClear', 
                'caretaker_lock' : 'WizzyLock'}

    with wizzy_sm:

        # start at 'clear' state
        smach.StateMachine.add('WizzyClear', WizzyClear(outcomes=transition_map.keys()), transitions=transition_map)

        smach.StateMachine.add('WizzyA', WizzyA(outcomes=transition_map.keys()), transitions=transition_map)
        smach.StateMachine.add('WizzyB', WizzyB(outcomes=transition_map.keys()), transitions=transition_map)
        smach.StateMachine.add('WizzyC', WizzyC(outcomes=transition_map.keys()), transitions=transition_map)
        
        smach.StateMachine.add('WizzyLock', WizzyLock(outcomes=transition_map.keys()), transitions=transition_map)

    outcome = wizzy_sm.execute()
