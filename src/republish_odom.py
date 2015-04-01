#!/usr/bin/env python

import rospy
import cv2

from math import sin,cos,pi

from spin_motors.msg import Encoder
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

class OdomCalculator():

    def __init__(self):
        rospy.init_node("odom_repub", anonymous=True)

        # state variables
        self.left_m = 0
        self.right_m = 0
        self.dx = None
        self.dr = None
        self.x = 0
        self.y = 0
        self.th = 0
        self.then = rospy.Time.now()

        # Parameters: frame IDs
        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_link')
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')

        # constants
        self.ticks_meter = float(rospy.get_param('~ticks_meter', 4*10000/(.1524*3.1415926))) # 4*ppr/circumference
        self.base_width = float(rospy.get_param('~base_width', .508))

        # things that do things
        self.encoderSub = rospy.Subscriber('encoder_count_rel', Encoder, self.calc_odom)
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size = 100)
        self.odomBroadcaster = TransformBroadcaster()




    def calc_odom(self, data):
        now = rospy.Time.now()
        elapsed = now - self.then
        self.then = now
        elapsed = elapsed.to_sec()

        enc_left, enc_right = data.left, data.right
        d_left = enc_left/self.ticks_meter #get meters travelled
        d_right = enc_right/self.ticks_meter #get meters travelled

        self.left_m += d_left
        self.right_m += d_right
        # rospy.loginfo("d_left: " + str(d_left) + " d_right: " + str(d_right))
        # rospy.loginfo("self.sum_left: " + str(self.sum_left) + " self.sum_right: " + str(self.sum_right))

        d = (d_left + d_right) / 2
        th = (d_right - d_left) / self.base_width
        self.dx = d / elapsed
        self.dr = th / elapsed
        # rospy.loginfo("d: " + str(d) + " elapsed: " + str(elapsed) + " self.dx: " + str(self.dx))

        if (d != 0):
            x = cos(th) * d
            y = -sin(th) * d
            self.x = self.x + ((cos(self.th) * x) - (sin(self.th) * y))
            self.y = self.y + ((sin(self.th) * x) + (cos(self.th) * y))
        if (th != 0):
            self.th = self.th + th

        # rospy.loginfo("self.x: " + str(self.x) + " self.y: " + str(self.y))

        # Publish odometry transform
        quaternion = Quaternion()
        quaternion.x = 0.0 
        quaternion.y = 0.0
        quaternion.z = sin(self.th / 2)
        quaternion.w = cos(self.th / 2)
        self.odomBroadcaster.sendTransform(
            (self.x, self.y, 0), 
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            rospy.Time.now(),
            self.base_frame_id,
            self.odom_frame_id
            )

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = self.dx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = self.dr
        self.odomPub.publish(odom)

        
if __name__ == '__main__':
    odom_calc = OdomCalculator()
    try:
        rospy.spin()
    except rospy.ROSInterruptException: 
        pass
