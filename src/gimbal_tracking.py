#!/usr/bin/env python
import rospy
import sensor_msgs
import vision_msgs
from vision_msgs.msg import Detection2DArray
from enum import Enum
import time
import threading
import signal
from std_srvs.srv import Trigger, TriggerResponse

import pid_controller
import hardware_serial as hw


class State(Enum):
    HOME = 0
    TRACKING = 1
    RETURN_TO_HOME = 2

class Gimbal_FSM:
    ''' State machine for the gimbal

    States:
        - Home
        - Tracking
        - Return_To_Home
    '''

    def __init__(self):
        self.state = State.RETURN_TO_HOME
        self.pid = pid_controller.Controller()
        self.tracking_timeout = float('inf')
        self.detection_time = float('inf')
    
    def track(self,measure_x,measure_y):
        rospy.loginfo("Tracking x=%s  y=%s" % (measure_x,measure_y))
        ref_x = 320 #center of image frame
        ref_y = 256
        error_x = ref_x - measure_x
        error_y = ref_y - measure_y
        u_yaw,u_pitch = self.pid.get_input(error_x,error_y)
        hw.speed_control(u_yaw,u_pitch)

    def return_to_home_position(self):
        hw.position_control(0,0)

    def stop(self):
        hw.speed_control(0,0)

    def get_set_rth_cb(self):
        def set_rth_cb(data): # data: Trigger
            self.state = State.RETURN+TO_HOME
            return TriggerResponse(success=True)
        return set_rth_cb


gimbal = Gimbal_FSM()

def cb_detection(data):
    ''' Callback from detection node
    '''

    if data.detections != []:
        
        x = data.detections[0].bbox.center.x #pixel in frame (0,640)
        y = data.detections[0].bbox.center.y #pixel in frame (0,512)
        
        global gimbal
        gimbal.state = State.TRACKING
        gimbal.detection_time = time.time()
        gimbal.track(x,y)



if __name__ == "__main__":
    rospy.init_node("gimbal_tracking")
    # TODO change to /superfluid/observations_out
    rospy.Subscriber("/superfluid/detections", Detection2DArray, cb_detection)
    rospy.Service(
        "/gimbal/return_to_home",
        Trigger,
        gimbal.get_set_rth_cb(),
    )
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        if gimbal.state == State.TRACKING:
            
            if time.time() - gimbal.detection_time > 10:  # return to home if timed out
                rospy.loginfo("TIMEOUT - Returning to home")
                gimbal.state = State.RETURN_TO_HOME
                gimbal.return_to_home_position()

            elif time.time() - gimbal.detection_time > 2: # stop motors if detection lost
                gimbal.stop()

        if gimbal.state == State.HOME:
            pass

        if gimbal.state == State.RETURN_TO_HOME:
            if abs(hw.get_encoder_angle_yaw()) < 1:
                rospy.loginfo("Arrived HOME")
                gimbal.state = State.HOME
                hw.follow_yaw_mode()
            else:
                gimbal.return_to_home_position()
                rospy.loginfo("Returning To Home")
        
        rate.sleep()


