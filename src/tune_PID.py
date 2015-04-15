#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

def control_pid():
    """ Main run loop for wall with slider """

    rospy.init_node('control_mode', anonymous=True)
    p_pub = rospy.Publisher('kp', Float32, queue_size=10)
    i_pub = rospy.Publisher('ki', Float32, queue_size=10)
    d_pub = rospy.Publisher('kd', Float32, queue_size=10)
    r = rospy.Rate(10)
    while not(rospy.is_shutdown()):
        try:
            params_dict = input("Enter constants in format {'p': val, 'i': val, 'd': val}:\n")
            p_pub.publish(Float32(params_dict['p']))
            i_pub.publish(Float32(params_dict['i']))
            d_pub.publish(Float32(params_dict['d']))
            r.sleep()
        except KeyboardInterrupt:
            break
        
if __name__ == '__main__':
    try:
        control_pid()
    except rospy.ROSInterruptException: pass
