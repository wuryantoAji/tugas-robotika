#! /usr/bin/env python

from __future__ import division
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np
import time
import sys

"""
Fungsi yang digunakan untuk melakukan invers kinematic
"""
def inverse_kinematics(trajectory_path):
    t0 = time.time()
    PI = math.pi
    UP = 0
    RIGHT = 1
    DOWN = 2
    LEFT = 3

    orientation = RIGHT

    for i in range(1, len(trajectory_path)-1):
        print("Action {} : Titik {} -> Titik {}".format(i,i,i+1))
        print("Orientation : {}".format(orientation))
        start_point = trajectory_path[i]
        xs, ys = start_point[0], start_point[1]
        finsih_point = trajectory_path[i+1]
        xf, yf = finsih_point[0], finsih_point[1]

        distx = abs(float(xf)-xs)
        disty = abs(float(yf)-ys)
        radius = min([distx, disty])
        distance = max([distx, disty]) - radius
        theta = PI
        move_time = 3
        
        if(distx == disty):
            print("Action yang diambil : Belok Sekali")
            # belok sekali
            t = round(time.time() - t0)
            v = get_linear_velocity_to_turn(radius, theta, move_time)
            w = get_angular_velocity(radius, theta, move_time)
            print("t = {}, v = {}, w = {}".format(t, v, w))
            if(orientation == UP):
                if(xf - xs > 0):
                    move(v, w, move_time)
                    orientation = turn_right(orientation)
                if(xf - xs < 0):
                    w = -w
                    move(v, w, move_time)
                    orientation = turn_left(orientation)
            elif(orientation == RIGHT):
                w = -w
                move(v, w, move_time)
                orientation = turn_left(orientation)
        elif(distx == 0):
            print("Action yang diambil : Belok 2 Kali")
            # belok 2 kali
            radius = disty * 0.5
            t = round(time.time() - t0)
            v = get_linear_velocity_to_turn(radius, theta, move_time)
            w = get_angular_velocity(radius, theta, move_time)
            print("t = {}, v = {}, w = {}".format(t, v, w))
            if(orientation == RIGHT):
                w = -w
                move(v, w, move_time)
                orientation = turn_left(orientation)
                move(v, w, move_time)
                orientation = turn_left(orientation)
            elif(orientation == LEFT):
                move(v, w, move_time)
                orientation = turn_right(orientation)
                move(v, w, move_time)
                orientation = turn_right(orientation)
        else:
            t = round(time.time() - t0)
            print("Action yang diambil : Maju dan Belok")
            # maju terus belok
            v = get_linear_velocity(distance, move_time)
            print("t = {}, v = {}, w = 0".format(t, v))
            move(v, 0, move_time)
            t = round(time.time() - t0)
            v = get_linear_velocity_to_turn(radius, theta, move_time)
            w = get_angular_velocity(radius, theta, move_time)
            print("t = {}, v = {}, w = {}".format(t, v, w))
            if(orientation == UP):
                if(xf - xs > 0):
                    move(v, w, move_time)
                    orientation = turn_right(orientation)
                if(xf - xs < 0):
                    w = -w
                    move(v, w, move_time)
                    orientation = turn_left(orientation)
            elif(orientation == RIGHT):
                    w = -w
                    move(v, w, move_time)
                    orientation = turn_left(orientation)
            elif(orientation == LEFT):
                    move(v, w, move_time)
                    orientation = turn_right(orientation)
        time.sleep(0.1)
        currx = round(odom.pose.pose.position.x, 1)
        curry = round(odom.pose.pose.position.y, 1)
        print("Posisi sekarang ({},{}) dari posisi seharusnya ({},{})".format(currx,curry,xf,yf))
        print
    t = round(time.time() - t0)
    print("Selesai dalam waktu {} detik".format(t))

"""
Fungsi yang digunakan untuk mencari kecepatan linear robot untuk berjalan lurus
Param :
    distance -> jarak tempuh robot
    move_time -> waktu bergerak robot dalam 1 action
"""
def get_linear_velocity(distance, move_time):
    return float(distance)/move_time

"""
Fungsi yang digunakan untuk mencari kecepatan linear robot untuk berbelok
Param :
    radius -> jari-jari dari perputaran robot
    theta -> sudut perputaran robot
    move_time -> waktu bergerak robot dalam 1 action
"""
def get_linear_velocity_to_turn(radius, theta, move_time):
    return (radius-((radius-1)/2))*(float(theta)/(2*move_time))

"""
Fungsi yang digunakan untuk mencari kecepatan angular robot
Param :
    theta -> sudut perputaran robot
    move_time -> waktu bergerak robot dalam 1 action
"""
def get_angular_velocity(radius, theta, move_time):
    return float(theta)/((radius-((radius-1)/2))*move_time)

"""
Fungsi yang digunakan untuk menggerakkan robot sesuai kecepatan dan waktu geraknya
Param :
    lin_vel -> kecepatan linear robot
    ang_vel -> kecepatan angular robot
    move_time -> waktu bergerak robot dalam 1 action
"""
def move(lin_vel, ang_vel, move_time):
    vel_pub = rospy.Publisher("/m4wr/cmd_vel", Twist, queue_size=10)

    vel = Twist()
    vel.linear.x = lin_vel
    vel.linear.y = 0
    vel.linear.z = 0
    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = ang_vel

    start_time = time.time()
    elapsed_time = 0
    relative_angle = 0

    if(ang_vel == 0):
        while(elapsed_time < move_time):
            vel_pub.publish(vel)
            now = time.time()
            elapsed_time = now - start_time
    else:
        while(relative_angle < math.pi):
            vel_pub.publish(vel)
            now = time.time()
            relative_angle = abs(ang_vel)*(now - start_time)

    
    vel.linear.x = 0
    vel.angular.z = 0
    vel_pub.publish(vel)

def turn_right(orientation):
    if(orientation==3):
        orientation = 0
        return orientation
    orientation += 1
    return orientation

def turn_left(orientation):
    if(orientation==0):
        orientation = 3
        return orientation
    orientation -= 1
    return orientation

trajectory_path = [
    (1, 0.5),
    (1, 1.5),
    (4, 2.5),
    (2.5, 4),
    (2.5, 6),
    (4, 7.5),
    (2.5, 11.5),
    (1, 12.5),
    (2.5, 14),
    (2.5, 16),
    (2.5, 18.5),
    (4, 19.5),
]

odom = None

def callback(msg):
    global odom
    odom = msg

if __name__ == "__main__":
    sub = rospy.Subscriber("/odom", Odometry, callback)
    rospy.init_node('scenario1_robot', anonymous=True)
    inverse_kinematics(trajectory_path)

