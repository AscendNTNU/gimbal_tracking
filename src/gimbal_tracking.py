#!/usr/bin/env python
import rospy
import sensor_msgs
import vision_msgs
from vision_msgs.msg import Detection2DArray
from enum import Enum
import time
import threading
import signal

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
        self.state = State.HOME
        self.pid = pid_controller.Controller()
        self.tracking_timeout = float('inf')
        self.detection_time = float('inf')
    
    def track(self,measure_x,measure_y):
        ref_x = 320 #center of image frame
        ref_y = 256
        error_x = ref_x - measure_x
        error_y = ref_y - measure_y
        u_yaw,u_pitch = self.pid.get_input(error_x,error_y)
        hw.speed_control(u_yaw,u_pitch)

    def return_to_home_position(self):
        print("TIMEOUT - Returning to home")
        hw.position_control(0,0)

    def stop(self):
        # print("TIMEOUT - Motor stopped")
        hw.speed_control(0,0)



gimbal = Gimbal_FSM()

def cb_detection(data):
    ''' Callback from detection node
    '''

    if data.detections != []:
        
        x = data.detections[0].bbox.center.x #pixel in frame (0,640)
        y = data.detections[0].bbox.center.y #pixel in frame (0,512)
        print("detected: x: %s  y: %s" % (x,y))
        
        global gimbal
        gimbal.state = State.TRACKING
        gimbal.detection_time = time.time()
        gimbal.track(x,y)
    
    else:
        if gimbal.state == State.TRACKING:
            
            if time.time() - gimbal.detection_time > 10:     # return to home if timed out
                gimbal.state = State.HOME
                gimbal.return_to_home_position()

            elif time.time() - gimbal.detection_time > 2: # stop if detection lost
                gimbal.stop()


if __name__ == "__main__":
    rospy.init_node("gimbal_tracking")
    rospy.Subscriber("/superfluid/detections",Detection2DArray,cb_detection)
    rospy.spin()


