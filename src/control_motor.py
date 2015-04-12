#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import Int64

distance_to_wall = -1
target = 0
pub = None

def set_target_speed(new_speed):
    """ call back function for the OpenCv Slider to set the target speed """
    global target
    target = new_speed - 1000

def control_speed():
    global pub
    """ Main run loop for wall with slider """
    cv2.namedWindow('UI')
    cv2.createTrackbar('speed (1000 = 0)', 'UI', target+1000, 2000, set_target_speed)
    rospy.init_node('control_mode', anonymous=True)
    pub = rospy.Publisher('motor_speed', Int64, queue_size=10)
    r = rospy.Rate(10)
    while not(rospy.is_shutdown()):
        cv2.waitKey(10)
        pub.publish(Int64(target))
        r.sleep()
        
if __name__ == '__main__':
    try:
        control_speed()
    except rospy.ROSInterruptException: pass
