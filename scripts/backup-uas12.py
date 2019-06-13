#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

import math

pub_ = None
regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
}
state_ = 'gerak_lurus'
state_dict_ = {
    'gerak_lurus': 'find the wall',
    'gerak_lurus_awas_kiri':'stabilize left',
    'gerak_lurus_awas_kanan':'stabilize right',
    'kiri': 'turn left',
    'kanan': 'turn right',
}

def clbk_laser(msg):
    global regions_
    #todo buat besok
    regions_ = {
        'right':  min(min(msg.ranges[0:20]), 10),
        'fright': min(min(msg.ranges[21:336]), 10),
        'front':  min(min(msg.ranges[337:377]), 10),
        'fleft':  min(min(msg.ranges[378:696]), 10),
        'left':   min(min(msg.ranges[697:713]), 10),
    }
    # regions_ = {
    #     'right':  min(min(msg.ranges[0:143]), 10),
    #     'fright': min(min(msg.ranges[144:287]), 10),
    #     'front':  min(min(msg.ranges[288:431]), 10),
    #     'fleft':  min(min(msg.ranges[432:575]), 10),
    #     'left':   min(min(msg.ranges[576:713]), 10),
    # }
    take_action()

def change_state(state):
    global state_, state_dict_
    if state is not state_:
        # print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state

def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    state_description = ''
    
    d = 1
    #kyknya ada yg kebalik
    if regions['front'] > d and regions['left'] > d and regions['right'] > d :
        state_description = 'case 1 - tidak ada halangan'
        print(state_description)
        change_state('gerak_lurus')
    elif regions['front'] < d and regions['left'] > d and regions['right'] > d:
        if regions['right'] > regions['left']:
            state_description = 'case 2 - ada halangan di depan, belok kanan'
            print(state_description)
            change_state('kanan')
        else:
            state_description = 'case 2 - ada halangan di depan, belok kiri'
            print(state_description)
            change_state('kiri')      
    elif regions['front'] < d and regions['left'] > d and regions['right'] < d:                    
        #di depan kanan ada halangan 
        state_description = 'case 5 - halangan di depan kanan belok kiri'
        print(state_description)
        change_state('kiri')
    elif regions['front'] < d and regions['left'] < d and regions['right'] > d:                    
        #di depan kiri ada halangan
        state_description = 'case 6 - halangan di depan kiri belok kanan'
        print(state_description)
        change_state('kanan')
    elif regions['front'] > d and regions['left'] < d and regions['right'] > d :                    
        #dikiri ada halangan
        if regions['fleft'] > 0.5 :
            state_description = 'case 3.1 - halangan dikiri'
            print(state_description)
            change_state('gerak_lurus')
        else :
            state_description = 'case 3.2 - halangan dikiri awas nabrak'
            print(state_description)
            change_state('gerak_lurus_awas_kiri')
                
    elif regions['front'] > d and regions['left'] > d and regions['right'] < d:                    
        #dikanan ada halangan
        if regions['fright'] > 0.5 :
            state_description = 'case 4.1 - halangan dikanan'
            print(state_description)
            change_state('gerak_lurus')
        else :
            state_description = 'case 4.2 - halangan dikanan awas nabrak'
            print(state_description)
            change_state('gerak_lurus_awas_kanan')

    else:
        state_description = 'unknown case'
        print(state_description)
        rospy.loginfo(regions)

def u_turn():
    PI = 3.1415926535897
    msg = Twist()
    angular_speed = 1*2*PI/360
    relative_angle = 90*2*PI/360
        t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < relative_angle):
        velocity_publisher.publish(msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)

    msg.angular.z = 0
    velocity_publisher.publish(msg)
    rospy.spin()


def find_wall():
    global regions_
    msg = Twist()
    msg.linear.x = 0.2
    # msg.angular.z = -0.3
    return msg
    
def turn_left():
    msg = Twist()
    msg.angular.z = -0.3
    return msg

def turn_right():
    msg = Twist()
    msg.angular.z = 0.3
    return msg

def awas_kiri():
    global regions_
    msg = Twist()
    msg.linear.x = 0.1
    msg.angular.z = 0.4
    return msg

def awas_kanan():
    global regions_
    msg = Twist()
    msg.linear.x = 0.1
    msg.angular.z = -0.4
    return msg

def main():
    global pub_
    
    rospy.init_node('reading_laser')
    
    pub_ = rospy.Publisher('/m4wr/cmd_vel', Twist, queue_size=1)
    
    sub = rospy.Subscriber('/m4wr/laser/scan', LaserScan, clbk_laser)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():

        msg = Twist()
        if state_ == 'gerak_lurus':
            msg = find_wall()
        elif state_ == 'kiri':
            msg = turn_left()
        elif state_ == 'kanan':
            msg = turn_right()
        elif state_ == 'gerak_lurus_awas_kiri':
            msg = awas_kiri()
        elif state_ == 'gerak_lurus_awas_kanan':
            msg = awas_kanan()        
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)

if __name__ == '__main__':
    main()
