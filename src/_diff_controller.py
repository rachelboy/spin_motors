#!/usr/bin/env python

"""
"""

import rospy
import threading

from math import sin,cos,pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import *
from babbage_motor_controller.msg import Control
from tf.broadcaster import TransformBroadcaster

from monitor_encoder import MonitorEncoder
from motor_packet import MotorPacket
from control_packet import ControlPacket

from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from babbage_motor_controller.cfg import MotorControllerConfig

###############################################################################
# Differential Controller Class
class DiffController:

    """ Controller to handle movement & odometry feedback for a differential
            drive mobile base. """

    def __init__(self, driver):
        self.driver = driver
        self.fake = driver.fake
        self.last_cmd = rospy.Time.now()

        # Parameters: networking
        self.ip_addr = rospy.get_param("~ip_addr", "10.2.98.200")
        self.motor_port = rospy.get_param("~motor_port", "6000")
        self.encoder_port = rospy.get_param("~encoder_port", "6001")
        self.control_port = rospy.get_param("~control_port", "6002")

        # Parameters: rates and geometry
        self.rate = rospy.get_param('~rate', 10.0)
        self.timeout = rospy.get_param('~timeout', 1.0)
        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        self.base_width = float(rospy.get_param('~base_width'))
        self.ticks_meter = float(rospy.get_param('~ticks_meter'))

        # Parameters: frame IDs
        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_link')
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')

        # Parameters: wheel orientation
        self.wheel_left_orientation = rospy.get_param('~wheel_left_orientation', -1)
        self.wheel_right_orientation = rospy.get_param('~wheel_right_orientation', 1)
        if self.wheel_left_orientation != -1:
            self.wheel_left_orientation = 1
        if self.wheel_right_orientation != -1:
            self.wheel_right_orientation = 1

        # Parameters: acceleration
        self.accel_limit = rospy.get_param('~accel_limit', 2.0)
        self.max_accel = int(self.accel_limit * self.ticks_meter / self.rate)

        # Output for joint states publisher
        self.joint_names = ["base_l_wheel_joint", "base_r_wheel_joint"]
        self.joint_positions = [0, 0]
        self.joint_velocities = [0, 0]

        # Internal state data
        self.v_left = 0                 # current setpoint velocity
        self.v_right = 0
        self.v_des_left = 0             # cmd_vel setpoint
        self.v_des_right = 0
        self.enc_left = None            # encoder readings
        self.enc_right = None
        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0
        self.dx = 0                     # speeds in x/rotation
        self.dr = 0
        self.sum_left = 0
        self.sum_right = 0
        self.then = rospy.Time.now()    # time for determining dx/dy

        # Dynamic reconfiguration 
        self.server = DynamicReconfigureServer(MotorControllerConfig, self.reconfigure)

        # Subscribers and publishers
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size = 10)
        self.controlPub = rospy.Publisher("control", Control, queue_size = 10)
        self.odomBroadcaster = TransformBroadcaster()

        # Setup control parameters
        self.control_packet = ControlPacket(self.ip_addr, self.control_port)

        # Setup motor commands
        self.motor_packet = MotorPacket(self.ip_addr, self.motor_port)

        # Setup encoder monitoring
        self.encoder_monitor = MonitorEncoder(None, self.encoder_callback, self.encoder_port)

        rospy.loginfo("Started DiffController. Geometry: " + str(self.base_width) + "m wide, " + str(self.ticks_meter) + " ticks/m.")

    def startup(self):
        if not self.fake:
            self.encoder_monitor.go()

    def shutdown(self):
        if not self.fake:
            self.encoder_monitor.stop()

    def update(self):
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()

            if self.fake:
                x = cos(self.th) * self.dx * elapsed
                y = -sin(self.th) * self.dx * elapsed
                self.x += cos(self.th) * self.dx * elapsed
                self.y += sin(self.th) * self.dx * elapsed
                self.th += self.dr * elapsed
            else:
                # Read latest encoder ticks
                left, right = self.get_encoder_ticks()

                if left != None and right != None:
                    # Calculate odometry
                    if self.enc_left == None:
                        d_left = 0
                        d_right = 0
                    else:
                       d_left = (left - self.enc_left) / self.ticks_meter
                       d_right = (right - self.enc_right) / self.ticks_meter
                    self.enc_left = left
                    self.enc_right = right
                    self.sum_left += d_left
                    self.sum_right += d_right
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

            if now > (self.last_cmd + rospy.Duration(self.timeout)):
                self.v_des_left = 0
                self.v_des_right = 0

            # Update control
            if not self.fake:
                self.control_count += 1
                if self.control_count >= self.rate:
                    self.control_count = 0
                    self.set_control_parameters()

            # Update motors
            if not self.fake:
                if self.v_left < self.v_des_left:
                    self.v_left += self.max_accel
                    if self.v_left > self.v_des_left:
                        self.v_left = self.v_des_left
                else:
                    self.v_left -= self.max_accel
                    if self.v_left < self.v_des_left:
                        self.v_left = self.v_des_left
                
                if self.v_right < self.v_des_right:
                    self.v_right += self.max_accel
                    if self.v_right > self.v_des_right:
                        self.v_right = self.v_des_right
                else:
                    self.v_right -= self.max_accel
                    if self.v_right < self.v_des_right:
                        self.v_right = self.v_des_right
                self.set_motor_velocities()

            self.t_next = now + self.t_delta

    def reconfigure(self, config, level):
        """ Handle dynamic reconfigure request """
        # Update the control configuration
        self.ff_factor = config["ff_factor"];
        self.ff_offset = config["ff_offset"];
        self.p_gain = config["p_gain"];
        self.d_gain = config["d_gain"];
        self.i_gain = config["i_gain"];
        self.i_max = config["i_max"];

        # Trigger an update
        self.control_count = self.rate

        return config

    def cmd_vel_callback(self, req):
        """ Handle movement request """
        # Save the time of this last command update
        self.last_cmd = rospy.Time.now()

        # Get the linear and angular velocities in the request
        linear_vel = req.linear.x      # m/s
        angular_vel = req.angular.z    # rad/s

        if self.fake:
            self.dx = linear_vel
            self.dr = angular_vel
        else:
            # Set motor velocities in ticks per second
            self.v_des_left = int(((linear_vel - (angular_vel * self.base_width / 2.0)) * self.ticks_meter))
            self.v_des_right = int(((linear_vel + (angular_vel * self.base_width / 2.0)) * self.ticks_meter))

    def encoder_callback(self):
        """ Encoder callback """
        # Prepare the control message
        control = Control()
        control.header.stamp = rospy.Time.now()
        control.header.frame_id = self.odom_frame_id

        # Get the control values
        control.left_command, control.right_command = self.encoder_monitor.get_command()
        control.left_velocity, control.right_velocity = self.encoder_monitor.get_velocity()
        control.left_feedforward, control.right_feedforward = self.encoder_monitor.get_feedforward()
        control.left_proportional, control.right_proportional = self.encoder_monitor.get_proportional()
        control.left_derivative, control.right_derivative = self.encoder_monitor.get_derivative()
        control.left_integrator, control.right_integrator = self.encoder_monitor.get_integrator()
        control.left_power, control.right_power = self.encoder_monitor.get_power()
        control.left_current, control.right_current = self.encoder_monitor.get_current()

        # Correct values for wheel orientation
        if self.wheel_left_orientation < 0:
            control.left_command = -control.left_command
            control.left_velocity = -control.left_velocity
            control.left_feedforward = -control.left_feedforward
            control.left_proportional = -control.left_proportional
            control.left_derivative = -control.left_derivative
            control.left_integrator = -control.left_integrator
            control.left_power = -control.left_power
            control.left_current = -control.left_current
        if self.wheel_right_orientation < 0:
            control.right_command = -control.right_command
            control.right_velocity = -control.right_velocity
            control.right_feedforward = -control.right_feedforward
            control.right_proportional = -control.right_proportional
            control.right_derivative = -control.right_derivative
            control.right_integrator = -control.right_integrator
            control.right_power = -control.right_power
            control.right_current = -control.right_current

        # Convert velocity values from ticks/second to meters/second
        control.left_command = control.left_command / self.ticks_meter
        control.right_command = control.right_command / self.ticks_meter
        control.left_velocity = control.left_velocity / self.ticks_meter
        control.right_velocity = control.right_velocity / self.ticks_meter

        # Scale the power related values -1.0 and 1.0
        control.left_feedforward = control.left_feedforward / 100.0
        control.right_feedforward = control.right_feedforward / 100.0
        control.left_proportional = control.left_proportional / 100.0
        control.right_proportional = control.right_proportional / 100.0
        control.left_derivative = control.left_derivative / 100.0
        control.right_derivative = control.right_derivative / 100.0
        control.left_integrator = control.left_integrator / 100.0
        control.right_integrator = control.right_integrator / 100.0
        control.left_power = control.left_power / 100.0
        control.right_power = control.right_power / 100.0

        # Publish the control message
        self.controlPub.publish(control)

    def get_encoder_ticks(self):
        """ Get latest signed 32-bit encoder tick counts """
        # Get the current encoder values
        encoder_left, encoder_right = self.encoder_monitor.get_count()

        # Correct encoder ticks for wheel orientation
        if encoder_left != None and self.wheel_left_orientation < 0:
            encoder_left = -encoder_left
        if encoder_right != None and self.wheel_right_orientation < 0:
            encoder_right = -encoder_right

        # Return the latest encoder ticks
        return [encoder_left, encoder_right]

    def set_motor_velocities(self):
        """ Set signed 32-bit motor velocities (ticks/second) """
        v_left = self.v_left 
        v_right = self.v_right

        # Correct velocities for orientation
        if self.wheel_left_orientation < 0:
            v_left = -v_left
        if self.wheel_right_orientation < 0:
            v_right = -v_right

        # rospy.loginfo("set_motor_velocities: " + str(v_left) + ", " + str(v_right))

        # Send the wheel velocity
        self.motor_packet.set_velocities(v_left, v_right)
        self.motor_packet.send()

    def set_control_parameters(self):
        """ Set PID control parameters """
        # rospy.loginfo("Updating control parameters: ")
        self.control_packet.set_ff_factor(self.ff_factor)
        self.control_packet.set_ff_offset(self.ff_offset)
        self.control_packet.set_p_gain(self.p_gain)
        self.control_packet.set_d_gain(self.d_gain)
        self.control_packet.set_i_gain(self.i_gain)
        self.control_packet.set_i_max(self.i_max)
        self.control_packet.send()

