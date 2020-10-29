#!/usr/bin/env python

import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf
from math import *
import time
import numpy as np


class Motor:
    def __init__(self, alpha):
        self.alpha = alpha
        self.prev_val = 0
        self.current_val = 0

    def filter(self, next_val):
        self.current_val = next_val * (1 - self.alpha) + self.prev_val * self.alpha
        self.prev_val = self.current_val
        return self.current_val


class RobotSimulator:
    def __init__(self, alpha, width, wheel_radius, dt=0.01):
        self.right_motor = Motor(alpha)
        self.left_motor = Motor(alpha)
        self.robot_width = width
        self.wheel_radius = wheel_radius
        self.max_wheel_vel = 10  # rad/s
        self.dt = dt

        # The following variables represent real values, not desired ones:
        self.linear_vel = 0
        self.angular_vel = 0
        self.theta = 0
        self.x = 0
        self.y = 0
        self.quaternion = [0, 0, 0, 1]

        self.odom = Odometry()

    def iterate_robot(self, desired_linear, desired_angular):
        desired_right, desired_left = self.robot_to_wheels(desired_linear, desired_angular)
        true_right, true_left = self.simulate_motors(desired_right, desired_left)
        real_linear_vel, real_angular_vel = self.wheels_to_robot(true_right, true_left)
        self.apply_displacement(real_linear_vel, real_angular_vel)

        # Following lines are for packing publisher objects:
        self.odom.pose.pose = Pose(Point(self.x, self.y, 0),
                                   Quaternion(self.quaternion[0], self.quaternion[1], self.quaternion[2], self.quaternion[3]))
        self.odom.twist.twist = Twist(Vector3(self.linear_vel, 0, 0), Vector3(0, 0, self.angular_vel))

    def simulate_motors(self, desired_right_vel, desired_left_vel):
        real_right_vel = self.right_motor.filter(desired_right_vel)
        real_left_vel = self.left_motor.filter(desired_left_vel)
        return real_right_vel, real_left_vel

    def robot_to_wheels(self, linear_vel, angular_vel):
        right_vel = (linear_vel / self.wheel_radius)-(angular_vel * self.robot_width)/(2*self.wheel_radius)
        left_vel = (linear_vel / self.wheel_radius)+(angular_vel * self.robot_width)/(2*self.wheel_radius)
        return right_vel, left_vel  # wheel angular vel, rad/s

    def wheels_to_robot(self, right_vel, left_vel):
        linear_vel = (left_vel + right_vel) * self.wheel_radius / 2
        angular_vel = (left_vel - right_vel) * self.wheel_radius / self.robot_width
        return linear_vel, angular_vel

    def apply_displacement(self, linear_vel, angular_vel):
        self.linear_vel = linear_vel
        self.angular_vel = angular_vel
        self.theta += angular_vel * self.dt
        self.x += linear_vel * cos(self.theta) * self.dt
        self.y += linear_vel * sin(self.theta) * self.dt
        self.quaternion = quaternion_from_euler(0, 0, self.theta)

    def override_tf(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        orientation = data.pose.pose.orientation  # For shorter typing

        self.quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        self.theta = euler_from_quaternion(self.quaternion)[2]


class CallbackHandler:
    def __init__(self):
        self.linear_command = 0
        self.angular_command = 0
        self.max_linear_velocity = 3
        self.max_angular_velocity = 3
        self.intake_extension = 0

    def drive_callback(self, data):
        self.angular_command = constrain_value(data.angular.z, -self.max_angular_velocity, self.max_angular_velocity)
        self.linear_command = constrain_value(data.linear.x, -self.max_linear_velocity, self.max_linear_velocity)

    def intake_callback(self, data):
        self.intake_extension = data.angular.y


def constrain_value(value, min_val, max_val):
    if value > max_val:
        value = max_val
    if value < min_val:
        value = min_val
    return value


if __name__ == '__main__':

    rospy.init_node('kobe_simulator')
    np.set_printoptions(suppress=True)

    robot = RobotSimulator(0.9, 0.64, 0.075)
    callback_object = CallbackHandler()
    vel_subscriber = rospy.Subscriber('/cmd_vel', Twist, callback_object.drive_callback)
    pose_subscriber = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, robot.override_tf)

    odom_broadcaster = tf.TransformBroadcaster()
    odom_publisher = rospy.Publisher('odom', Odometry, queue_size=100)
    odom = Odometry()
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_footprint'  # base_link is rotated because of solid-to-urdf converter

    robotJoints = JointState()
    robotJoints.name = ['main_body_to_intake', 'main_body_to_turret']

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        robot.iterate_robot(callback_object.linear_command, callback_object.angular_command)
        current_time = rospy.Time.now()

        # Broadcast tf:
        odom_broadcaster.sendTransform((robot.x, robot.y, 0), robot.quaternion, current_time, 'base_footprint', 'odom')

        # Publish odometry:
        # odom.header.stamp = current_time
        # odom_quat = robot.quaternion  # Just for shorter typing
        # odom.pose.pose = Pose(Point(robot.x, robot.y, 0),
        #                       Quaternion(odom_quat[0], odom_quat[1], odom_quat[2], odom_quat[3]))
        # odom.twist.twist = Twist(Vector3(robot.linear_vel, 0, 0), Vector3(0, 0, robot.angular_vel))  # Intrinsic speed
        odom_publisher.publish(robot.odom)

        rate.sleep()  # Wait fot 0.01 secs
