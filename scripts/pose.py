#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
import time

"""
Made Wira Dhanar Santika
1606880996
"""

def callback(model_states):
    rospy.loginfo(rospy.get_caller_id() + '{}\n'.format(model_states.pose))
    

"""
Fungsi ini digunakan untuk melakukan subscribe posisi robot di gazebo.
Topic yang digunakan adalah /gazebo/model-states
"""
def listener():

    rospy.init_node('robot_listener', anonymous=True)

    rospy.Subscriber('/gazebo/model_states', ModelStates, callback)

    # Biarkan program berjalan hingga di stop
    rospy.spin()

if __name__ == '__main__':
    listener()
