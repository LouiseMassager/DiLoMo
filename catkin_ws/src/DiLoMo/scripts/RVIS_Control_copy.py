#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


def talker():

    pub = rospy.Publisher('joint_space', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(10)  # 10hz
    hello_str = JointState()
    hello_str.header = Header()
    hello_str.header.stamp = rospy.Time.now()
    hello_str.name = ['arm_left_joint_1', 'arm_left_joint_2', 'arm_left_joint_3', 'arm_left_joint_4',
                      'arm_left_joint_5', 'arm_left_joint_6', 'arm_left_joint_7', 'arm_fight_joint_1', 'arm_fight_joint_2', 'arm_fight_joint_3',
                      'arm_fight_joint_4', 'arm_fight_joint_5', 'arm_fight_joint_6', 'arm_fight_joint_7', 'leg_left_joint_1', 'leg_left_joint_2',
                      'leg_left_joint_3', 'leg_left_joint_4', 'leg_left_joint_5', 'leg_right_joint_1', 'leg_right_joint_2', 'leg_right_joint_3',
                      'leg_right_joint_4', 'leg_right_joint_5', 'head_joint', 'neck_joint', 'body_joint']
    hello_str.position = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    hello_str.position = []
    hello_str.velocity = []
    hello_str.effort = []

    while not rospy.is_shutdown():
        hello_str.header.stamp = rospy.Time.now()
        pub.publish(hello_str)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
