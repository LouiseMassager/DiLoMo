#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def talker(): 
    pub = rospy.Publisher('/dilomo/joint_space', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(10)  # 10hz
    hello_str = JointState()
    hello_str.header = Header()
    hello_str.header.stamp = rospy.Time.now()
    hello_str.name = ['joint_space']
    hello_str.position = [1,0,0]
    hello_str.velocity = [1,1,1]
    hello_str.effort = [1,1,1]

    while not rospy.is_shutdown():
        hello_str.header.stamp = rospy.Time.now()
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
