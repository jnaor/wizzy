#!/usr/bin/env python
"""
myDM - My small Decision Making algorithm
       Temporary place holder for minimal Lidar and USB relay control
"""
import rospy   
from std_msgs.msg       import String, Float64

class myDM :

    def __init__ (self) : 
        self.DM_state = "DM_RUN"
        self.start_time = 0
        self.prev_lidar_state = "INIT"
        
        self.TEMPORARY_TIME = (2.0) # Seconds
        rospy.Subscriber ('/myLidar/state', String, self.state_cb)
        
        # Publish
        self.usb_relay_cmd  = rospy.Publisher ('/usb_relay_command',    String, queue_size=10)        

    def state_cb (self, msg) :
        lidar_state = msg.data
        if lidar_state != "OK" :
            self.usb_relay_cmd.publish("on")
        else :
            self.usb_relay_cmd.publish("off")
        # if self.prev_lidar_state != lidar_state :
            # print ("==> Changed from lidar_state " + self.prev_lidar_state + " to " + lidar_state)
            # self.prev_lidar_state = lidar_state
        
        # if self.DM_state == "DM_RUN" :
            # if lidar_state != "OK" :
                # self.usb_relay_cmd.publish("on")
                # self.DM_state = "DM_IMMOBILIZE_TEMPORARY"
                # self.start_time = rospy.get_time()                
            # else :
                # self.usb_relay_cmd.publish("off")
        # elif self.DM_state == "DM_IMMOBILIZE_TEMPORARY" :
            # curr_time = rospy.get_time()
            # if ((curr_time - self.start_time) > self.TEMPORARY_TIME) :
                # self.DM_state = "DM_RUN_TEMPORARY"
                # self.start_time = rospy.get_time() 
        # elif self.DM_state == "DM_RUN_TEMPORARY" :
            # self.usb_relay_cmd.publish("off")
            # curr_time = rospy.get_time()
            # if ((curr_time - self.start_time) > self.TEMPORARY_TIME) :
                # self.DM_state = "DM_RUN"
       
if __name__ == '__main__':
    rospy.init_node('myDM')
    my_DM = myDM()
    rospy.spin()
    
        
        
        
        