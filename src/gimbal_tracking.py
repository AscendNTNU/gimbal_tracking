#!/usr/bin/env python
import rospy
import sensor_msgs
import vision_msgs
from vision_msgs.msg import Detection2DArray

import pid_controller
from hardware_serial import move_gimbal

pid = pid_controller.Controller()


def cb_detection(data):
    stamp_sec = data.header.stamp.secs
    if data.detections != []:
        width = data.detections[0].source_img.width
        height = data.detections[0].source_img.height
        x = data.detections[0].bbox.center.x #bredde pixelkoordinat (0,640)
        y = data.detections[0].bbox.center.y #hoyde pixelkoordinat (0,512)

        print("detected: x: %s  y: %s" % (x,y))
        global pid
        u_yaw,u_pitch = pid.get_input(x,y)
        move_gimbal(u_yaw,u_pitch)
        






if __name__ == "__main__":
    rospy.init_node("gimbal_tracking")
    rospy.Subscriber("/superfluid/detections",Detection2DArray,cb_detection)
    rospy.spin() 
