#!/usr/bin/env python

import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
import numpy as np
import time


if __name__ == '__main__':
	np.set_printoptions(suppress=True)

	rospy.init_node('ros_training_publisher')

	sinus_publisher = rospy.Publisher('signal/sinus', Float32, queue_size=100)
	vector_publisher = rospy.Publisher('my_vector', Vector3Stamped, queue_size=100)

	message_header = Header()
	message_header.frame_id = 'doesnt matter right now'

	rate = rospy.Rate(100)  # 100 Hz -> 100 times per second

	while not rospy.is_shutdown():

		sinus_to_send = np.sin(time.time())

		current_time = rospy.Time.now()
		message_header.stamp = current_time

		# vector_to_send = Vector3Stamped()
		# vector_to_send.header = message_header
		# vector_to_send.vector.x = sinus_to_send

		vector_to_send = Vector3Stamped(message_header,Vector3(sinus_to_send, 0, 0))
		vector_to_send.vector.z = np.arcsin(sinus_to_send)

		sinus_publisher.publish(sinus_to_send)		# input argument has to match the type declared when creating the publisher!
		vector_publisher.publish(vector_to_send)	# input argument has to match the type declared when creating the publisher!

		rate.sleep() # Wait
