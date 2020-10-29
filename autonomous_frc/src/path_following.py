#!/usr/bin/env python
from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from math import *
from tf.transformations import euler_from_quaternion
import rospy

#TODO add reverse

def sign(num):
    if num < 0:
        return -1
    else:
        return 1


class PathFollowing:
    def __init__(self):
        self.kv = 1.0
        self.k_theta = 4.0
        self.k_omega = 0.5
        self.k_curvature = 1.0
        self.angle_tolerance = 0.05  # radians
        self.range_tolerance = 0.2  # meters
        self.max_velocity = 2.0  # m/s

        self.path = Path()
        self.drive_command = Twist()
        self.odom = Odometry()
        self.command_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=50)
        self.finish_publisher = rospy.Publisher('path_finished', Bool, queue_size=50)

        self.point_index = 1
        self.path_length = 0
        self.reverse = True
        self.is_active = False

    def calc_mid_errors(self):
        my_heading = self.extract_heading(self.odom.pose.pose.orientation)
        if self.reverse:
            my_heading += 3.14159

        my_point = self.odom.pose.pose.position
        target_point_current = self.path.poses[self.point_index].pose.position
        target_point_prev = self.path.poses[self.point_index-1].pose.position

        error_position_current = sqrt((my_point.x - target_point_current.x) ** 2 + (my_point.y - target_point_current.y) ** 2)
        error_position_prev = sqrt((my_point.x - target_point_prev.x) ** 2 + (my_point.y - target_point_prev.y) ** 2)
        target_direction = atan2((target_point_current.y - my_point.y), (target_point_current.x - my_point.x))

        error_theta = target_direction - my_heading
        if error_theta > 3.14159:
            error_theta -= 6.2832
        if error_theta < -3.14159:
            error_theta += 6.2832

        return error_position_current, error_position_prev, error_theta

    def calc_last_error(self):
        my_heading = self.extract_heading(self.odom.pose.pose.orientation)
        if self.reverse:
            my_heading += 3.14159
        final_heading = self.extract_heading(self.path.poses[self.path_length-1].pose.orientation)

        error_theta = final_heading - my_heading
        if error_theta > 3.14159:
            error_theta -= 6.2832
        if error_theta <= -3.14159:
            error_theta += 6.2832

        # print my_heading, final_heading, error_theta
        return error_theta

    def iterate_follow(self):
        omega = self.odom.twist.twist.angular.z  # This line executes anyway
        if self.point_index <= self.path_length - 1:
            # Distance_current is the distance from the current goal.
            # Distance_perv is the distance from the previous point.
            distance_current, distance_prev, error_theta = self.calc_mid_errors()
            signal_angular = error_theta * self.k_theta - omega * self.k_omega
            # if self.point_index == self.path_length - 1:

            my_point = self.odom.pose.pose.position
            target_point = self.path.poses[-1].pose.position
            last_dist = sqrt((my_point.x - target_point.x) ** 2 + (my_point.y - target_point.y) ** 2)
            if last_dist < 0.5:
                signal_linear = distance_current * self.kv
                if distance_current < self.range_tolerance:
                    self.point_index += 1
            else:
                signal_linear = self.max_velocity - abs(error_theta * self.k_curvature)
                if distance_current < distance_prev:
                    self.point_index += 1

            signal_linear = max(signal_linear, 0)
            # Even though I kinda like the reverse when making sharp turns ;)
            if self.reverse:
                signal_linear *= -1
            self.drive_command.angular.z = signal_angular
            self.drive_command.linear.x = signal_linear


        else:
            self.drive_command.linear.x = 0
            error_theta = self.calc_last_error()
            signal_angular = error_theta * self.k_theta - omega * self.k_omega
            self.drive_command.angular.z = signal_angular
            if abs(error_theta) < self.angle_tolerance:
                self.drive_command.angular.z = 0
                self.is_active = False
                self.finish_publisher.publish(Bool())

        self.command_publisher.publish(self.drive_command)

    def extract_heading(self, quat):
        yaw_heading = euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))[2]
        return yaw_heading

    def stop(self):
        self.drive_command.angular.z = 0
        self.drive_command.linear.x = 0
        self.command_publisher.publish(self.drive_command)

    def path_callback(self, path):
        self.path = path
        self.point_index = 1
        self.path_length = len(path.poses)
        self.reverse = rospy.get_param('reverse')
        self.is_active = True

    def odom_callback(self, odom):
        self.odom = odom

    def pose_callback(self, data):
        self.is_active = False


if __name__ == '__main__':
    rospy.init_node('path_following')
    if not rospy.has_param('reverse'):
        rospy.set_param('reverse', False)

    path_follower = PathFollowing()
    rate = rospy.Rate(50)
    path_subscriber = rospy.Subscriber('path', Path, path_follower.path_callback)
    odom_subscriber = rospy.Subscriber('odom', Odometry, path_follower.odom_callback)
    pose_subscriber = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, path_follower.pose_callback)

    while not rospy.is_shutdown():
        if path_follower.is_active:
            path_follower.iterate_follow()
        else:
            path_follower.stop()
        rate.sleep()
