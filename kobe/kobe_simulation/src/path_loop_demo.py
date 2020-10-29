#!/usr/bin/env python

# from cubic_spline import spline
from std_msgs.msg import *
from geometry_msgs.msg import *
import time
from math import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import rospy
import numpy as np

class CallbackHandler:
    def __init__(self):
        self.goals = [{'x': 0, 'y': 5.5, 'theta': -180, 'reverse': False, 'shoot': 0},
                      {'x': -3, 'y': 5.5, 'theta': -180, 'reverse': False, 'shoot': 0},
                      {'x': -0.4, 'y': 0.3, 'theta': -20, 'reverse': True, 'shoot': 5},
                      {'x': -2, 'y': -1.3, 'theta': -180, 'reverse': False, 'shoot': 0},
                      {'x': -5, 'y': -1.5, 'theta': -180, 'reverse': False, 'shoot': 0},
                      {'x': -0.5, 'y': 0.2, 'theta': 0, 'reverse': True, 'shoot': 3}]

        self.current_goal = PoseStamped()
        self.current_index = 0
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=50)

    def path_finished_callback(self, data):
        time.sleep(0.5)
        rospy.set_param('reverse', self.goals[self.current_index]['reverse'])
        balls_to_shoot = self.goals[self.current_index]['shoot']
        if balls_to_shoot>0:
            self.shoot_balls(balls_to_shoot)

        self.generate_pose(self.current_index)
        self.goal_publisher.publish(self.current_goal)

        self.current_index += 1
        if self.current_index == len(self.goals):
            self.current_index = 0

    def generate_pose(self, index):
        self.current_goal.pose.position.x = self.goals[index]['x']
        self.current_goal.pose.position.y = self.goals[index]['y']
        angle_radians = radians(self.goals[index]['theta'])
        quat = quaternion_from_euler(0, 0, angle_radians)
        self.current_goal.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3],)
        self.current_goal.header.stamp = rospy.Time.now()

    def shoot_balls(self, amount):
        # For now, this is a placeholder...
        pass


if __name__ == '__main__':
    np.set_printoptions(suppress=True)
    rospy.init_node('path_looper')
    if not rospy.has_param('reverse'):
        rospy.set_param('reverse', False)

    callback_object = CallbackHandler()
    finish_subscriber = rospy.Subscriber('path_finished', Bool, callback_object.path_finished_callback)

    # For some reason, the first publishing doesn't get recognized if it comes too fast:
    time.sleep(0.5)
    callback_object.path_finished_callback(True)

    while not rospy.is_shutdown():
        rospy.spin()
