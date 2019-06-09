#!/usr/bin/env python

"""
Made Wira Dhanar Santika
1606880996
"""

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf.transformations
import math
import numpy as np
import sys

PI = math.pi
odom = None
speed = 0.5

destX = 0
destY = 0

"""
Fungsi move akan membuat robot bergerak searah
garis lurus sepanjang input.
Topic yang digunakan adalah /cmd_vel sesuai dengan file
xacro yang tersedia
"""
def move(speed, destX, destY):

    startX = odom.pose.pose.position.x
    startY = odom.pose.pose.position.y

    quaternion = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]

    relativeDestX = destX - startX
    relativeDestY = destY - startY

    rotation_matrix = np.array([[np.cos(yaw), np.sin(yaw)],
                                [-np.sin(yaw), np.cos(yaw)]])
    destination_vector = np.array([[relativeDestX],
                                    [relativeDestY]])
    new_destination = rotation_matrix.dot(destination_vector)
    relativeDestX = new_destination[0][0]
    relativeDestY = new_destination[1][0]

    distance = math.sqrt(math.pow(relativeDestX,2) + math.pow(relativeDestY,2))
    angle_rotation = np.arctan(relativeDestY/relativeDestX)

    rotate(angle_rotation)

    velocity_publisher = rospy.Publisher('/m4wr/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    
    vel_msg.linear.x = abs(speed)
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    
    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    while(current_distance < distance):
        velocity_publisher.publish(vel_msg)
        t1=rospy.Time.now().to_sec()
        current_distance= speed*(t1-t0)

    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)

"""
Fungsi stop digunakan untuk menghentikan gerakan robot
"""
def stop():

    velocity_publisher = rospy.Publisher('/m4wr/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    
    velocity_publisher.publish(vel_msg)

"""
Fungsi rotate akan membuat robot berbelok sebanyak
sudut yang diinginkan. Sudut dalam derajat.
Topic yang digunakan adalah /cmd_vel sesuai dengan file
xacro yang tersedia
"""
def rotate(angle):
    velocity_publisher = rospy.Publisher('/m4wr/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    speed = angle

    angular_speed = speed
    relative_angle = angle

    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = abs(angular_speed)

    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)

    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

def callback(msg):
    global odom
    odom = msg

if __name__ == '__main__':
    rospy.init_node('laserscan_robot', anonymous=True)
    while(True):
        try:
            sub = rospy.Subscriber("/odom", Odometry, callback)
            destX = float(raw_input("Masukkan koordinat x tujuan: "))
            destY = float(raw_input("Masukkan koordinat y tujuan: "))
            move(speed, destX, destY)
        except (KeyboardInterrupt, SystemExit):
            stop()
            print("Should be exitted")
            raise
