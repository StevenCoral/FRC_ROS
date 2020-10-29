#! /usr/bin/env python
import rospy, xbox
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def start():
    # print('sdfghjsdfghj\n\n\n\nasdgfhjk')

    rospy.init_node('joy_fucking_stick')
    rate= rospy.Rate(30)
    command_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=30)
    joy=xbox.Joystick()
    # x,y = 0,1
    drive_command = Twist()
    while not rospy.is_shutdown():
        x = joy.leftX() 
        y = joy.leftY()
        # print 'x: ', x, 'y: ', y
        drive_command.linear.x = y
        drive_command.angular.z = -x * sign(y)
        command_publisher.publish(drive_command)
        rate.sleep()

start()
