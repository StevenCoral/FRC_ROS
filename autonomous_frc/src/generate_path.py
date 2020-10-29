#!/usr/bin/env python

# from cubic_spline import spline
from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import  *
from sensor_msgs.msg import *
from nav_msgs.msg import  *
from math import *
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import rospy
import numpy as np
import matplotlib.pyplot as plt


class PathGenerator:
    def __init__(self):
        self.x = None
        self.y = None
        self.conversion_matrix = np.linalg.inv(np.array([[0, 0, 0, 1],
                                                         [0, 0, 1, 0],
                                                         [1, 1, 1, 1],
                                                         [3, 2, 1, 0]]))

    def calc_coeffs(self, pos_begin, angle_begin, pos_end, angle_end):
        condition_vector = np.array([pos_begin, angle_begin, pos_end, angle_end])
        desired_coeffs = self.conversion_matrix.dot(condition_vector)
        return desired_coeffs

    def create_path(self, first_point, last_point, k, number_of_points=100):
        x_coeffs = self.calc_coeffs(first_point[0],
                                    np.cos(radians(first_point[2])) * k,
                                    last_point[0],
                                    np.cos(radians(last_point[2])) * k)
        xd_coeffs = np.polyder(x_coeffs)
        xdd_coeffs = np.polyder(xd_coeffs)

        y_coeffs = self.calc_coeffs(first_point[1],
                                    np.sin(radians(first_point[2])) * k,
                                    last_point[1],
                                    np.sin(radians(last_point[2])) * k)
        yd_coeffs = np.polyder(y_coeffs)
        ydd_coeffs = np.polyder(yd_coeffs)

        s = np.linspace(0, 1, number_of_points)
        self.x = np.polyval(x_coeffs, s)
        xd = np.polyval(xd_coeffs, s)
        xdd = np.polyval(xdd_coeffs, s)
        self.y = np.polyval(y_coeffs, s)
        yd = np.polyval(yd_coeffs, s)
        ydd = np.polyval(ydd_coeffs, s)
        curvature = np.abs(xd*ydd-yd*xdd) / (xd**2 + yd**2)**1.5

        path = []
        for segment in range(number_of_points):
            point = [self.x[segment], self.y[segment], np.arctan2(yd[segment], xd[segment]), curvature[segment]]
            path.append(point)

        return path


class CallbackHandler:
    # The values here are not mandatory, we can choose to initialize whatever we want.
    # I just gave a few things as an example. You better move on to the main function and come back here later:
    def __init__(self):
        self.header = Header()
        self.x = 0
        self.y = 0
        self.theta = 0
        self.point = [0, 0, 0]
        self.generator = PathGenerator()
        self.path_publisher = rospy.Publisher('/path', Path, queue_size=20)
        self.path = Path()
        self.path.header.frame_id = "/map"

    def nav_goal_callback(self, data):
        x_goal = data.pose.position.x
        y_goal = data.pose.position.y
        distance = sqrt((self.x-x_goal)**2+(self.y-y_goal)**2)
        quaternion = data.pose.orientation
        goal_heading = euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        theta_goal = degrees(goal_heading[2])
        theta_start = self.theta

        reverse = rospy.get_param('reverse')
        if reverse:
            theta_start += 180

        number_of_points = rospy.get_param('number_of_path_points')
        points = self.generator.create_path([self.x, self.y, theta_start],
                                            [x_goal, y_goal, theta_goal],
                                            distance, number_of_points)
        poses_list = []
        for p in points:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "/map"
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            # Cant add curvature because Rviz draws it as Z in the world
            # pose.pose.position.z = p[3]
            quat = quaternion_from_euler(0, 0, p[2])
            pose.pose.orientation = Quaternion(0, 0, quat[2], quat[3])  # Heading
            poses_list.append(pose)

        self.path.poses = poses_list
        self.path.header.stamp = rospy.Time.now()
        self.path_publisher.publish(self.path)

    def odom_callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        quaternion = data.pose.pose.orientation
        my_heading = euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        self.theta = degrees(my_heading[2])


if __name__ == '__main__':
    np.set_printoptions(suppress=True)
    rospy.init_node('path_generator')
    if not rospy.has_param('number_of_path_points'):
        rospy.set_param('number_of_path_points', 10)
    if not rospy.has_param('reverse'):
        rospy.set_param('reverse', False)

    callback_object = CallbackHandler()
    nav_goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_object.nav_goal_callback)
    odom_subscriber = rospy.Subscriber('/odom', Odometry, callback_object.odom_callback)

    while not rospy.is_shutdown():
        rospy.spin()
