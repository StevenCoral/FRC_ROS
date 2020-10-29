#!/usr/bin/env python

import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
import numpy as np
import time

class callback_functions:
    def __init__(self):
        self.vector = Vector3Stamped()
        self.header = Header()

    def sinus_callback(self, data):
        if rospy.get_param('print_sinus'):
            print data.data * rospy.get_param('sinus_amplitude')

    def vector_callback(self, data):
        self.header = data.header
        self.vector = data.vector


if __name__ == '__main__':
    np.set_printoptions(suppress=True)

    rospy.init_node('ros_training_subscriber')

    # It is good practice to do the following for every param you use in the node, setting default values:
    if not rospy.has_param('print_vector'):
        rospy.set_param('print_vector', False)
    if not rospy.has_param('print_sinus'):
        rospy.set_param('print_sinus', False)
    if not rospy.has_param('sinus_amplitude'):
        rospy.set_param('sinus_amplitude', 1.0)
    
    function_object = callback_functions()

    sinus_subscriber = rospy.Subscriber('signal/sinus', Float32, function_object.sinus_callback)
    vector_subscriber = rospy.Subscriber('/my_vector', Vector3Stamped, function_object.vector_callback)

    #rospy.spin()  # This is like 'while True', and can be used instead of the loop if node is ONLY listening
    rate = rospy.Rate(2)  # 2 Hz -> 2 times per second

    while not rospy.is_shutdown():

        if rospy.get_param('print_vector'):
            vector = function_object.vector  # just to show options, this one makes the next line shorter...
            print (function_object.header.frame_id, vector.x, vector.y, vector.z)
            # Can also write "print header" or so, but will get it line by line as in "rostopic echo"
        rate.sleep()
        
