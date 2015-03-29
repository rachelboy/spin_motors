#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import cv2
from std_msgs.msg import Int64

distance_to_wall = -1
target = 0
p=0
i=10
d=0
pub = None

def set_target_speed(new_speed):
    """ call back function for the OpenCv Slider to set the target speed """
    global target
    target = new_speed-1000

def set_p(new_p):
    """ call back function for the OpenCv Slider to set the target speed """
    global p
    p = new_p

def set_i(new_i):
    """ call back function for the OpenCv Slider to set the target speed """
    global i
    i = new_i

def set_d(new_d):
    """ call back function for the OpenCv Slider to set the target speed """
    global d
    d = new_d

def control_speed():
    global pub
    """ Main run loop for wall with slider """
    cv2.namedWindow('UI')
    cv2.createTrackbar('speed', 'UI', target+1000, 2000, set_target_speed)
    cv2.createTrackbar('P', 'UI', p, 250, set_p)
    cv2.createTrackbar('I', 'UI', i, 250, set_i)
    cv2.createTrackbar('D', 'UI', d, 250, set_d)

    rospy.init_node('control_mode', anonymous=True)
    s_pub = rospy.Publisher('motor_speed', Int64, queue_size=10)
    p_pub = rospy.Publisher('p_value', Int64, queue_size=10)
    i_pub = rospy.Publisher('i_value', Int64, queue_size=10)
    d_pub = rospy.Publisher('d_value', Int64, queue_size=10)
    r = rospy.Rate(10)
    while not(rospy.is_shutdown()):
        cv2.waitKey(10)
        s_pub.publish(Int64(target))
        p_pub.publish(Int64(p))
        i_pub.publish(Int64(i))
        d_pub.publish(Int64(d))
        r.sleep()
        
if __name__ == '__main__':
    try:
        control_speed()
    except rospy.ROSInterruptException: pass
