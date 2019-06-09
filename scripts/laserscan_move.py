#!/usr/bin/env python

"""
Made Wira Dhanar Santika
1606880996
"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import sys

PI = math.pi
laser_regions = []
speed = 0.5

"""
Fungsi move akan membuat robot bergerak searah
garis lurus sepanjang input.
Topic yang digunakan adalah /cmd_vel sesuai dengan file
xacro yang tersedia
"""
def move(speed):

    if(not laser_regions):
        return

    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    
    vel_msg.linear.x = abs(speed)
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    
    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    while(current_distance < 0.1):
        velocity_publisher.publish(vel_msg)
        t1=rospy.Time.now().to_sec()
        current_distance= speed*(t1-t0)

    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)

    if min(laser_regions) < 0.7:
        rotate(90)

"""
Fungsi stop digunakan untuk menghentikan gerakan robot
"""
def stop():

    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
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
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    speed = angle

    angular_speed = speed*2*PI/360
    relative_angle = angle*2*PI/360

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
    global laser_regions
    regions = [
        min(msg.ranges[0:143]),
        min(msg.ranges[144:287]),
        min(msg.ranges[288:431]),
        min(msg.ranges[432:575]),
        min(msg.ranges[576:713]),
    ]
    laser_regions = regions

if __name__ == '__main__':
    rospy.init_node('laserscan_robot', anonymous=True)
    while(True):
        try:
            sub = rospy.Subscriber("/praktikum_2/laser/scan", LaserScan, callback)
            move(speed)
        except (KeyboardInterrupt, SystemExit):
            stop()
            print("Should be exitted")
            raise
