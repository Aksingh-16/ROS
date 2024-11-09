#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time

def move_straight(velocity_publisher, speed, duration):
    vel_msg = Twist()
    vel_msg.linear.x = speed
    vel_msg.angular.z = 0.0
    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < duration:
        velocity_publisher.publish(vel_msg)
    vel_msg.linear.x = 0.0
    velocity_publisher.publish(vel_msg)

def rotate(velocity_publisher, angular_speed, angle):
    vel_msg = Twist()
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = angular_speed
    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < angle / angular_speed:
        velocity_publisher.publish(vel_msg)
    vel_msg.angular.z = 0.0
    velocity_publisher.publish(vel_msg)

def move_rectangle():
    rospy.init_node('move_turtle_rectangle_node', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    # Dimensions of the rectangle
    length = 2.0  # meters
    breadth = 1.0  # meters

    while not rospy.is_shutdown():
        move_straight(velocity_publisher, 1.0, length)  # Move along length
        rotate(velocity_publisher, 1.0, 1.57)         # 90-degree turn
        move_straight(velocity_publisher, 1.0, breadth)  # Move along breadth
        rotate(velocity_publisher, 1.0, 1.57)         # 90-degree turn

if __name__ == '__main__':
    try:
        move_rectangle()
    except rospy.ROSInterruptException:
        pass
