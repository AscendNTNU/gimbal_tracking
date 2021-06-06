#!/usr/bin/env python
import rospy
import sensor_msgs
import vision_msgs
from vision_msgs.msg import Detection2DArray
from enum import Enum
import time
import threading

import pid_controller
from hardware_serial import move_gimbal


class State(Enum):
    HOME = 0
    TRACKING = 1

class Gimbal_FSM:

    def __init__(self):
        self.state = State.HOME
        self.pid = pid_controller.Controller()
        self.home_timeout = time.time()
    
    # TODO tracking state
    def track(self,measure_x,measure_y):
        u_yaw,u_pitch = self.pid.get_input(measure_x,measure_y)
        move_gimbal(u_yaw,u_pitch)

    def return_to_home_position(self):
        print("Returning to home")
        move_gimbal(0,0)

    # TODO home state


        
gimbal = Gimbal_FSM()
lock = threading.Lock()
def cb_detection(data):
    
    # TODO if != []: set state=track
    # reset target_timout
    # reset home_timeout
    # -> do operations

    if data.detections != []:
        x = data.detections[0].bbox.center.x #bredde pixelkoordinat (0,640)
        y = data.detections[0].bbox.center.y #hoyde pixelkoordinat (0,512)
        print("detected: x: %s  y: %s" % (x,y))
        
        lock.acquire()
        global gimbal
        gimbal.state = State.TRACKING
        # gimbal.target_timeout = 3 #seconds
        gimbal.home_timeout = time.time()
        lock.release()

        gimbal.track(x,y)



if __name__ == "__main__":
    rospy.init_node("gimbal_tracking")
    rospy.Subscriber("/superfluid/detections",Detection2DArray,cb_detection)
    

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        
        lock.acquire()
        global gimbal
        if gimbal.state == State.TRACKING:
            if time.time() - gimbal.home_timeout > 10:
                print("delta time is: "+str(time.time() - gimbal.home_timeout)+"sec")
                gimbal.state = State.HOME
                lock.release()
                
                gimbal.return_to_home_position()
        
        if lock.locked():
            lock.release()
        
        rate.sleep()

