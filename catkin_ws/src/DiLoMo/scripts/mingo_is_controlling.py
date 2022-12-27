#!/usr/bin/env python
#import time
import rospy
from math import pi, sin, cos, acos
import random
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

"""
Topics To Write on:
type: std_msgs/Float64
/dilomo/arm_left_joint_1_position_controller/command
/dilomo/arm_left_joint_2_position_controller/command
/dilomo/arm_left_joint_3_position_controller/command
/dilomo/arm_left_joint_4_position_controller/command
/dilomo/arm_left_joint_5_position_controller/command
/dilomo/arm_left_joint_6_position_controller/command
/dilomo/arm_left_joint_7_position_controller/command
/dilomo/arm_fight_joint_1_position_controller/command
/dilomo/arm_fight_joint_2_position_controller/command
/dilomo/arm_fight_joint_3_position_controller/command
/dilomo/arm_fight_joint_4_position_controller/command
/dilomo/arm_fight_joint_5_position_controller/command
/dilomo/arm_fight_joint_6_position_controller/command
/dilomo/arm_fight_joint_7_position_controller/command
/dilomo/leg_left_joint_1_position_controller/command
/dilomo/leg_left_joint_2_position_controller/command
/dilomo/leg_left_joint_3_position_controller/command
/dilomo/leg_left_joint_4_position_controller/command
/dilomo/leg_left_joint_5_position_controller/command
/dilomo/leg_right_joint_1_position_controller/command
/dilomo/leg_right_joint_2_position_controller/command
/dilomo/leg_right_joint_3_position_controller/command
/dilomo/leg_right_joint_4_position_controller/command
/dilomo/leg_right_joint_5_position_controller/command
/dilomo/head_joint_position_controller/command
/dilomo/neck_joint_position_controller/command
/dilomo/body_joint_position_controller/command
"""


class dilomoJointMover(object):

    def __init__(self):

        rospy.init_node('jointmover_demo', anonymous=True)
        rospy.loginfo("dilomo JointMover Initialising...")
        self.pub_dilomo_arm_left_joint_1_position = rospy.Publisher('/dilomo/arm_left_joint_1_position_controller/command',
                                                                    Float64,
                                                                    queue_size=1)
        self.pub_dilomo_arm_left_joint_2_position = rospy.Publisher('/dilomo/arm_left_joint_2_position_controller/command',
                                                                    Float64,
                                                                    queue_size=1)
        self.pub_dilomo_arm_left_joint_3_position = rospy.Publisher('/dilomo/arm_left_joint_3_position_controller/command',
                                                                    Float64,
                                                                    queue_size=1)
        self.pub_dilomo_arm_left_joint_4_position = rospy.Publisher('/dilomo/arm_left_joint_4_position_controller/command',
                                                                    Float64,
                                                                    queue_size=1)
        self.pub_dilomo_arm_left_joint_5_position = rospy.Publisher('/dilomo/arm_left_joint_5_position_controller/command',
                                                                    Float64,
                                                                    queue_size=1)
        self.pub_dilomo_arm_left_joint_6_position = rospy.Publisher('/dilomo/arm_left_joint_6_position_controller/command',
                                                                    Float64,
                                                                    queue_size=1)
        self.pub_dilomo_arm_left_joint_7_position = rospy.Publisher('/dilomo/arm_left_joint_7_position_controller/command',
                                                                    Float64,
                                                                    queue_size=1)
        self.pub_dilomo_arm_right_joint_1_position = rospy.Publisher('/dilomo/arm_right_joint_1_position_controller/command',
                                                                     Float64,
                                                                     queue_size=1)
        self.pub_dilomo_arm_right_joint_2_position = rospy.Publisher('/dilomo/arm_right_joint_2_position_controller/command',
                                                                     Float64,
                                                                     queue_size=1)
        self.pub_dilomo_arm_right_joint_3_position = rospy.Publisher('/dilomo/arm_right_joint_3_position_controller/command',
                                                                     Float64,
                                                                     queue_size=1)
        self.pub_dilomo_arm_right_joint_4_position = rospy.Publisher('/dilomo/arm_right_joint_4_position_controller/command',
                                                                     Float64,
                                                                     queue_size=1)
        self.pub_dilomo_arm_right_joint_5_position = rospy.Publisher('/dilomo/arm_right_joint_5_position_controller/command',
                                                                     Float64,
                                                                     queue_size=1)
        self.pub_dilomo_arm_right_joint_6_position = rospy.Publisher('/dilomo/arm_right_joint_6_position_controller/command',
                                                                     Float64,
                                                                     queue_size=1)
        self.pub_dilomo_arm_right_joint_7_position = rospy.Publisher('/dilomo/arm_right_joint_7_position_controller/command',
                                                                     Float64,
                                                                     queue_size=1)
        self.pub_dilomo_leg_left_joint_1_position = rospy.Publisher('/dilomo/leg_left_joint_1_position_controller/command',
                                                                    Float64,
                                                                    queue_size=1)
        self.pub_dilomo_leg_left_joint_2_position = rospy.Publisher('/dilomo/leg_left_joint_2_position_controller/command',
                                                                    Float64,
                                                                    queue_size=1)
        self.pub_dilomo_leg_left_joint_3_position = rospy.Publisher('/dilomo/leg_left_joint_3_position_controller/command',
                                                                    Float64,
                                                                    queue_size=1)
        self.pub_dilomo_leg_left_joint_4_position = rospy.Publisher('/dilomo/leg_left_joint_4_position_controller/command',
                                                                    Float64,
                                                                    queue_size=1)
        self.pub_dilomo_leg_left_joint_5_position = rospy.Publisher('/dilomo/leg_left_joint_5_position_controller/command',
                                                                    Float64,
                                                                    queue_size=1)
        self.pub_dilomo_leg_right_joint_1_position = rospy.Publisher('/dilomo/leg_right_joint_1_position_controller/command',
                                                                     Float64,
                                                                     queue_size=1)
        self.pub_dilomo_leg_right_joint_2_position = rospy.Publisher('/dilomo/leg_right_joint_2_position_controller/command',
                                                                     Float64,
                                                                     queue_size=1)
        self.pub_dilomo_leg_right_joint_3_position = rospy.Publisher('/dilomo/leg_right_joint_3_position_controller/command',
                                                                     Float64,
                                                                     queue_size=1)
        self.pub_dilomo_leg_right_joint_4_position = rospy.Publisher('/dilomo/leg_right_joint_4_position_controller/command',
                                                                     Float64,
                                                                     queue_size=1)
        self.pub_dilomo_leg_right_joint_5_position = rospy.Publisher('/dilomo/leg_right_joint_5_position_controller/command',
                                                                     Float64,
                                                                     queue_size=1)
        self.pub_dilomo_head_joint_position = rospy.Publisher('/dilomo/head_joint_position_controller/command',
                                                              Float64,
                                                              queue_size=1)
        self.pub_dilomo_body_joint_position = rospy.Publisher('/dilomo/body_joint_position_controller/command',
                                                              Float64,
                                                              queue_size=1)
        self.pub_dilomo_neck_joint_position = rospy.Publisher('/dilomo/neck_joint_position_controller/command',
                                                              Float64,
                                                              queue_size=1)

        joint_states_topic_name = "/dilomo/joint_states"
        rospy.Subscriber(joint_states_topic_name, JointState,
                         self.dilomo_joints_callback)
        dilomo_joints_data = None
        conter = 0
        while dilomo_joints_data is None:
            conter = conter+1
            try:
                dilomo_joints_data = rospy.wait_for_message(
                    joint_states_topic_name, JointState, timeout=5)
            except:
                rospy.logwarn("Time out " + str(joint_states_topic_name))
                pass
            if conter > 10:
                conter = 0
                break

        self.dilomo_joint_dictionary = dict(
            zip(dilomo_joints_data.name, dilomo_joints_data.position))

    def move_dilomo_all_joints(self, arm_left_1, arm_left_2, arm_left_3, arm_left_4, arm_left_5, arm_left_6, arm_left_7,
                               arm_right_1, arm_right_2, arm_right_3, arm_right_4, arm_right_5, arm_right_6, arm_right_7, leg_left_1, leg_left_2,
                               leg_left_3, leg_left_4, leg_left_5, leg_right_1, leg_right_2, leg_right_3, leg_right_4, leg_right_5, head, body, neck):
        angle_arm_left_1 = Float64()
        angle_arm_left_1.data = arm_left_1
        angle_arm_left_2 = Float64()
        angle_arm_left_2.data = arm_left_2
        angle_arm_left_3 = Float64()
        angle_arm_left_3.data = arm_left_3
        angle_arm_left_4 = Float64()
        angle_arm_left_4.data = arm_left_4
        angle_arm_left_5 = Float64()
        angle_arm_left_5.data = arm_left_5
        angle_arm_left_6 = Float64()
        angle_arm_left_6.data = arm_left_6
        angle_arm_left_7 = Float64()
        angle_arm_left_7.data = arm_left_7

        angle_arm_right_1 = Float64()
        angle_arm_right_1.data = arm_right_1
        angle_arm_right_2 = Float64()
        angle_arm_right_2.data = arm_right_2
        angle_arm_right_3 = Float64()
        angle_arm_right_3.data = arm_right_3
        angle_arm_right_4 = Float64()
        angle_arm_right_4.data = arm_right_4
        angle_arm_right_5 = Float64()
        angle_arm_right_5.data = arm_right_5
        angle_arm_right_6 = Float64()
        angle_arm_right_6.data = arm_right_6
        angle_arm_right_7 = Float64()
        angle_arm_right_7.data = arm_right_7

        angle_leg_left_1 = Float64()
        angle_leg_left_1.data = leg_left_1
        angle_leg_left_2 = Float64()
        angle_leg_left_2.data = leg_left_2
        angle_leg_left_3 = Float64()
        angle_leg_left_3.data = leg_left_3
        angle_leg_left_4 = Float64()
        angle_leg_left_4.data = leg_left_4
        angle_leg_left_5 = Float64()
        angle_leg_left_5.data = leg_left_5

        angle_leg_right_1 = Float64()
        angle_leg_right_1.data = leg_right_1
        angle_leg_right_2 = Float64()
        angle_leg_right_2.data = leg_right_2
        angle_leg_right_3 = Float64()
        angle_leg_right_3.data = leg_right_3
        angle_leg_right_4 = Float64()
        angle_leg_right_4.data = leg_right_4
        angle_leg_right_5 = Float64()
        angle_leg_right_5.data = leg_right_5

        angle_head = Float64()
        angle_head.data = head
        angle_body = Float64()
        angle_body.data = body
        angle_neck = Float64()
        angle_neck.data = neck

        self.pub_dilomo_arm_left_joint_1_position.publish(angle_arm_left_1)
        self.pub_dilomo_arm_left_joint_2_position.publish(angle_arm_left_2)
        self.pub_dilomo_arm_left_joint_3_position.publish(angle_arm_left_3)
        self.pub_dilomo_arm_left_joint_4_position.publish(angle_arm_left_4)
        self.pub_dilomo_arm_left_joint_5_position.publish(angle_arm_left_5)
        self.pub_dilomo_arm_left_joint_6_position.publish(angle_arm_left_6)
        self.pub_dilomo_arm_left_joint_7_position.publish(angle_arm_left_7)

        self.pub_dilomo_arm_right_joint_1_position.publish(angle_arm_right_1)
        self.pub_dilomo_arm_right_joint_2_position.publish(angle_arm_right_2)
        self.pub_dilomo_arm_right_joint_3_position.publish(angle_arm_right_3)
        self.pub_dilomo_arm_right_joint_4_position.publish(angle_arm_right_4)
        self.pub_dilomo_arm_right_joint_5_position.publish(angle_arm_right_5)
        self.pub_dilomo_arm_right_joint_6_position.publish(angle_arm_right_6)
        self.pub_dilomo_arm_right_joint_7_position.publish(angle_arm_right_7)

        self.pub_dilomo_leg_left_joint_1_position.publish(angle_leg_left_1)
        self.pub_dilomo_leg_left_joint_2_position.publish(angle_leg_left_2)
        self.pub_dilomo_leg_left_joint_3_position.publish(angle_leg_left_3)
        self.pub_dilomo_leg_left_joint_4_position.publish(angle_leg_left_4)
        self.pub_dilomo_leg_left_joint_5_position.publish(angle_leg_left_5)

        self.pub_dilomo_leg_right_joint_1_position.publish(angle_leg_right_1)
        self.pub_dilomo_leg_right_joint_2_position.publish(angle_leg_right_2)
        self.pub_dilomo_leg_right_joint_3_position.publish(angle_leg_right_3)
        self.pub_dilomo_leg_right_joint_4_position.publish(angle_leg_right_4)
        self.pub_dilomo_leg_right_joint_5_position.publish(angle_leg_right_5)

        self.pub_dilomo_head_joint_position.publish(angle_head)
        self.pub_dilomo_body_joint_position.publish(angle_body)
        self.pub_dilomo_neck_joint_position.publish(angle_neck)

    def move_arm_left_joint_1(self, position):
        """
        limits radians : lower="-3.14" upper="3.14"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_dilomo_arm_left_joint_1_position.publish(angle)

    def move_arm_left_joint_2(self, position):
        """
        limits radians : lower="-3.14" upper="3.14"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_dilomo_arm_left_joint_2_position.publish(angle)

    def move_arm_left_joint_3(self, position):
        """
        limits radians : lower="-3.14" upper="3.14"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_dilomo_arm_left_joint_3_position.publish(angle)

    def move_arm_left_joint_4(self, position):
        """
        limits radians : lower="-3.14" upper="3.14"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_dilomo_arm_left_joint_4_position.publish(angle)

    def move_arm_left_joint_5(self, position):
        """
        limits radians : lower="-3.14" upper="3.14"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_dilomo_arm_left_joint_5_position.publish(angle)

    def move_arm_left_joint_6(self, position):
        """
        limits radians : lower="-3.14" upper="3.14"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_dilomo_arm_left_joint_6_position.publish(angle)

    def move_arm_left_joint_7(self, position):
        """
        limits radians : lower="-3.14" upper="3.14"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_dilomo_arm_left_joint_7_position.publish(angle)

    def move_arm_right_joint_1(self, position):
        """
        limits radians : lower="-3.14" upper="3.14"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_dilomo_arm_right_joint_1_position.publish(angle)

    def move_arm_right_joint_2(self, position):
        """
        limits radians : lower="-3.14" upper="3.14"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_dilomo_arm_right_joint_2_position.publish(angle)

    def move_arm_right_joint_3(self, position):
        """
        limits radians : lower="-3.14" upper="3.14"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_dilomo_arm_right_joint_3_position.publish(angle)

    def move_arm_right_joint_4(self, position):
        """
        limits radians : lower="-3.14" upper="3.14"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_dilomo_arm_right_joint_4_position.publish(angle)

    def move_arm_right_joint_5(self, position):
        """
        limits radians : lower="-3.14" upper="3.14"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_dilomo_arm_right_joint_5_position.publish(angle)

    def move_arm_right_joint_6(self, position):
        """
        limits radians : lower="-3.14" upper="3.14"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_dilomo_arm_right_joint_6_position.publish(angle)

    def move_arm_right_joint_7(self, position):
        """
        limits radians : lower="-3.14" upper="3.14"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_dilomo_arm_right_joint_1_position.publish(angle)

    def move_leg_left_joint_1(self, position):
        """
        limits radians : lower="-3.14" upper="3.14"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_dilomo_leg_left_joint_1_position.publish(angle)

    def move_leg_left_joint_2(self, position):
        """
        limits radians : lower="-3.14" upper="3.14"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_dilomo_leg_left_joint_2_position.publish(angle)

    def move_leg_left_joint_3(self, position):
        """
        limits radians : lower="-3.14" upper="3.14"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_dilomo_leg_left_joint_3_position.publish(angle)

    def move_leg_left_joint_4(self, position):
        """
        limits radians : lower="-3.14" upper="3.14"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_dilomo_leg_left_joint_4_position.publish(angle)

    def move_leg_left_joint_5(self, position):
        """
        limits radians : lower="-3.14" upper="3.14"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_dilomo_leg_left_joint_5_position.publish(angle)

    def move_leg_right_joint_1(self, position):
        """
        limits radians : lower="-3.14" upper="3.14"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_dilomo_leg_right_joint_1_position.publish(angle)

    def move_leg_right_joint_2(self, position):
        """
        limits radians : lower="-3.14" upper="3.14"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_dilomo_leg_right_joint_2_position.publish(angle)

    def move_leg_right_joint_3(self, position):
        """
        limits radians : lower="-3.14" upper="3.14"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_dilomo_leg_right_joint_3_position.publish(angle)

    def move_leg_right_joint_4(self, position):
        """
        limits radians : lower="-3.14" upper="3.14"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_dilomo_leg_right_joint_4_position.publish(angle)

    def move_leg_right_joint_5(self, position):
        """
        limits radians : lower="-3.14" upper="3.14"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_dilomo_leg_right_joint_5_position.publish(angle)

    def move_head_joint(self, position):
        """
        limits radians : lower="-3.14" upper="3.14"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_dilomo_head_joint_position.publish(angle)

    def move_neck_joint(self, position):
        """
        limits radians : lower="-3.14" upper="3.14"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_dilomo_neck_joint_position.publish(angle)

    def move_body_joint(self, position):
        """
        limits radians : lower="-3.14" upper="3.14"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_dilomo_body_joint_position.publish(angle)

    def dilomo_joints_callback(self, msg):
        """
        sensor_msgs/JointState
        std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
        string[] name
        float64[] position
        float64[] velocity
        float64[] effort

        :param msg:
        :return:
        """
        self.dilomo_joint_dictionary = dict(zip(msg.name, msg.position))

    def dilomo_check_joint_value(self, joint_name, value, error=0.1):
        """
        Check the joint by name 'arm_left_joint_1', 'arm_left_joint_2', 'arm_left_joint_3', 'arm_left_joint_4',
        'arm_left_joint_5', 'arm_left_joint_6', 'arm_left_joint_7', 'arm_fight_joint_1', 'arm_fight_joint_2', 'arm_fight_joint_3',
        'arm_fight_joint_4', 'arm_fight_joint_5', 'arm_fight_joint_6', 'arm_fight_joint_7', 'leg_left_joint_1', 'leg_left_joint_2',
        'leg_left_joint_3', 'leg_left_joint_4', 'leg_left_joint_5', 'leg_right_joint_1', 'leg_right_joint_2', 'leg_right_joint_3',
        'leg_right_joint_4', 'leg_right_joint_5', 'head_joint', 'neck_joint', 'body_joint' is near the value given
        :param value:
        :return:
        """
        similar = self.dilomo_joint_dictionary.get(joint_name) >= (
            value - error) and self.dilomo_joint_dictionary.get(joint_name) <= (value + error)

        return similar

    def convert_angle_to_unitary(self, angle):
        """
        Removes complete revolutions from angle and converts to the positive equivalent
        if the angle is negative
        :param angle: Has to be in radians
        :return:
        """
        # Convert to angle between [0,360)
        complete_rev = 2 * pi
        mod_angle = int(angle / complete_rev)
        clean_angle = angle - mod_angle * complete_rev
        # Convert Negative angles to their corresponding positive values
        if clean_angle < 0:
            clean_angle += 2 * pi

        return clean_angle

    def assertAlmostEqualAngles(self, x, y,):
        c2 = (sin(x) - sin(y)) ** 2 + (cos(x) - cos(y)) ** 2
        angle_diff = acos((2.0 - c2) / 2.0)
        return angle_diff

    def dilomo_check_continuous_joint_value(self, joint_name, value, error=100):
        """
        Check the joint by name 'arm_left_joint_1', 'arm_left_joint_2', 'arm_left_joint_3', 'arm_left_joint_4',
        'arm_left_joint_5', 'arm_left_joint_6', 'arm_left_joint_7', 'arm_fight_joint_1', 'arm_fight_joint_2', 'arm_fight_joint_3',
        'arm_fight_joint_4', 'arm_fight_joint_5', 'arm_fight_joint_6', 'arm_fight_joint_7', 'leg_left_joint_1', 'leg_left_joint_2',
        'leg_left_joint_3', 'leg_left_joint_4', 'leg_left_joint_5', 'leg_right_joint_1', 'leg_right_joint_2', 'leg_right_joint_3',
        'leg_right_joint_4', 'leg_right_joint_5', 'head_joint', 'neck_joint', 'body_joint' is near the value given
        We have to convert the joint values removing whole revolutions and converting negative versions
        of the same angle
        :param value:
        :return:
        """
        joint_reading = self.dilomo_joint_dictionary.get(joint_name)
        clean_joint_reading = self.convert_angle_to_unitary(
            angle=joint_reading)
        clean_value = self.convert_angle_to_unitary(angle=value)

        dif_angles = self.assertAlmostEqualAngles(
            clean_joint_reading, clean_value)
        similar = dif_angles <= error

        return similar

    def dilomo_stand(self, arm_left_1, arm_left_2, arm_left_3, arm_left_4, arm_left_5, arm_left_6, arm_left_7,
                     arm_right_1, arm_right_2, arm_right_3, arm_right_4, arm_right_5, arm_right_6, arm_right_7,
                     leg_left_1, leg_left_2, leg_left_3, leg_left_4, leg_left_5, leg_right_1, leg_right_2, leg_right_3,
                     leg_right_4, leg_right_5, head, body, neck):
        """
        Make dilomo look down
        :return:
        """
        check_rate = 5.0
        #conter = 0

        position_arm_left_1 = arm_left_1
        position_arm_left_2 = arm_left_2
        position_arm_left_3 = arm_left_3
        position_arm_left_4 = arm_left_4
        position_arm_left_5 = arm_left_5
        position_arm_left_6 = arm_left_6
        position_arm_left_7 = arm_left_7
        position_arm_right_1 = arm_right_1
        position_arm_right_2 = arm_right_2
        position_arm_right_3 = arm_right_3
        position_arm_right_4 = arm_right_4
        position_arm_right_5 = arm_right_5
        position_arm_right_6 = arm_right_6
        position_arm_right_7 = arm_right_7
        position_leg_left_1 = leg_left_1
        position_leg_left_2 = leg_left_2
        position_leg_left_3 = leg_left_3
        position_leg_left_4 = leg_left_4
        position_leg_left_5 = leg_left_5
        position_leg_right_1 = leg_right_1
        position_leg_right_2 = leg_right_2
        position_leg_right_3 = leg_right_3
        position_leg_right_4 = leg_right_4
        position_leg_right_5 = leg_right_5
        position_head = head
        position_body = body
        position_neck = neck

        similar_arm_left_1 = False
        similar_arm_left_2 = False
        similar_arm_left_3 = False
        similar_arm_left_4 = False
        similar_arm_left_5 = False
        similar_arm_left_6 = False
        similar_arm_left_7 = False
        similar_arm_right_1 = False
        similar_arm_right_2 = False
        similar_arm_right_3 = False
        similar_arm_right_4 = False
        similar_arm_right_5 = False
        similar_arm_right_6 = False
        similar_arm_right_7 = False
        similar_leg_left_1 = False
        similar_leg_left_2 = False
        similar_leg_left_3 = False
        similar_leg_left_4 = False
        similar_leg_left_5 = False
        similar_leg_right_1 = False
        similar_leg_right_2 = False
        similar_leg_right_3 = False
        similar_leg_right_4 = False
        similar_leg_right_5 = False

        similar_head = False
        similar_body = False
        similar_neck = False

        rate = rospy.Rate(check_rate)

        while not (similar_arm_left_1 and similar_arm_left_2 and similar_arm_left_3 and similar_arm_left_4 and similar_arm_left_5 and similar_arm_left_6 and similar_arm_left_7
                   and similar_arm_right_1 and similar_arm_right_2 and similar_arm_right_3 and similar_arm_right_4 and similar_arm_right_5 and similar_arm_right_6 and similar_arm_right_7
                   and similar_leg_left_1 and similar_leg_left_2 and similar_leg_left_3 and similar_leg_left_4 and similar_leg_left_5
                   and similar_leg_right_1 and similar_leg_right_2 and similar_leg_right_3 and similar_leg_right_4 and similar_leg_right_5
                   and similar_head and similar_body and similar_neck):

            self.move_dilomo_all_joints(position_arm_left_1, position_arm_left_2, position_arm_left_3, position_arm_left_4, position_arm_left_5, position_arm_left_6, position_arm_right_7,
                                        position_arm_right_1, position_arm_right_2, position_arm_right_3, position_arm_right_4, position_arm_right_5, position_arm_right_6, position_arm_right_7,
                                        position_leg_left_1, position_leg_left_2, position_leg_left_3, position_leg_left_4, position_leg_left_5,
                                        position_leg_right_1, position_leg_right_2, position_leg_right_3, position_leg_right_4, position_leg_right_5,
                                        position_neck, position_body, position_head)

            similar_arm_left_1 = self.dilomo_check_continuous_joint_value(
                joint_name="arm_left_joint_1", value=position_arm_left_1)
            similar_arm_left_2 = self.dilomo_check_continuous_joint_value(
                joint_name="arm_left_joint_2", value=position_arm_left_2)
            similar_arm_left_3 = self.dilomo_check_continuous_joint_value(
                joint_name="arm_left_joint_3", value=position_arm_left_3)
            similar_arm_left_4 = self.dilomo_check_continuous_joint_value(
                joint_name="arm_left_joint_4", value=position_arm_left_4)
            similar_arm_left_5 = self.dilomo_check_continuous_joint_value(
                joint_name="arm_left_joint_5", value=position_arm_left_5)
            similar_arm_left_6 = self.dilomo_check_continuous_joint_value(
                joint_name="arm_left_joint_6", value=position_arm_left_6)
            similar_arm_left_7 = self.dilomo_check_continuous_joint_value(
                joint_name="arm_left_joint_7", value=position_arm_left_7)

            similar_arm_right_1 = self.dilomo_check_continuous_joint_value(
                joint_name="arm_right_joint_1", value=position_arm_right_1)
            similar_arm_right_2 = self.dilomo_check_continuous_joint_value(
                joint_name="arm_right_joint_2", value=position_arm_right_2)
            similar_arm_right_3 = self.dilomo_check_continuous_joint_value(
                joint_name="arm_right_joint_3", value=position_arm_right_3)
            similar_arm_right_4 = self.dilomo_check_continuous_joint_value(
                joint_name="arm_right_joint_4", value=position_arm_right_4)
            similar_arm_right_5 = self.dilomo_check_continuous_joint_value(
                joint_name="arm_right_joint_5", value=position_arm_right_5)
            similar_arm_right_6 = self.dilomo_check_continuous_joint_value(
                joint_name="arm_right_joint_6", value=position_arm_right_6)
            similar_arm_right_7 = self.dilomo_check_continuous_joint_value(
                joint_name="arm_right_joint_7", value=position_arm_right_7)

            similar_leg_left_1 = self.dilomo_check_continuous_joint_value(
                joint_name="leg_left_joint_1", value=position_leg_left_1)
            similar_leg_left_2 = self.dilomo_check_continuous_joint_value(
                joint_name="leg_left_joint_2", value=position_leg_left_2)
            similar_leg_left_3 = self.dilomo_check_continuous_joint_value(
                joint_name="leg_left_joint_3", value=position_leg_left_3)
            similar_leg_left_4 = self.dilomo_check_continuous_joint_value(
                joint_name="leg_left_joint_4", value=position_leg_left_4)
            similar_leg_left_5 = self.dilomo_check_continuous_joint_value(
                joint_name="leg_left_joint_5", value=position_leg_left_5)

            similar_leg_right_1 = self.dilomo_check_continuous_joint_value(
                joint_name="leg_right_joint_1", value=position_leg_right_1)
            similar_leg_right_2 = self.dilomo_check_continuous_joint_value(
                joint_name="leg_right_joint_2", value=position_leg_right_2)
            similar_leg_right_3 = self.dilomo_check_continuous_joint_value(
                joint_name="leg_right_joint_3", value=position_leg_right_3)
            similar_leg_right_4 = self.dilomo_check_continuous_joint_value(
                joint_name="leg_right_joint_4", value=position_leg_right_4)
            similar_leg_right_5 = self.dilomo_check_continuous_joint_value(
                joint_name="leg_right_joint_5", value=position_leg_right_5)

            similar_head = self.dilomo_check_continuous_joint_value(
                joint_name="head_joint", value=position_head)
            similar_body = self.dilomo_check_continuous_joint_value(
                joint_name="body_joint", value=position_body)
            similar_neck = self.dilomo_check_continuous_joint_value(
                joint_name="neck_joint", value=position_neck)

            #conter = conter+1
            #if conter > 200:
                #rospy.loginfo("Break happens...")
                #conter = 0
                #break

            rate.sleep()

    def dilomo_default(self):
        self.dilomo_stand(
            arm_left_1=0, arm_left_2=0, arm_left_3=0, arm_left_4=0, arm_left_5=0, arm_left_6=0, arm_left_7=1.57,
            arm_right_1=0, arm_right_2=0, arm_right_3=0, arm_right_4=0, arm_right_5=0, arm_right_6=0, arm_right_7=1.57,
            leg_left_1=0, leg_left_2=0, leg_left_3=0, leg_left_4=0, leg_left_5=0, leg_right_1=0,
            leg_right_2=0, leg_right_3=0, leg_right_4=0, leg_right_5=0,
            neck=0.1,body=0,head=0.1)

    def dilomo_deform(self):
        self.dilomo_stand(arm_left_1=1.57, arm_left_2=0, arm_left_3=0, arm_left_4=1.57, arm_left_5=0, arm_left_6=0, arm_left_7=0,
                          arm_right_1=-1.57, arm_right_2=0, arm_right_3=0, arm_right_4=-1.57, arm_right_5=0, arm_right_6=0, arm_right_7=0,
                          leg_left_1=0, leg_left_2=1.57, leg_left_3=0, leg_left_4=0, leg_left_5=1.31,
                          leg_right_1=0, leg_right_2=1.57, leg_right_3=0, leg_right_4=0, leg_right_5=1.31,
                          head=0, body=0, neck=0)

    
    def deform_arms(self):
   
        check_rate = 5.0
        position = 1.57
        

        rate = rospy.Rate(check_rate)
        for repetition in range(2):
            similar_1 = False
            similar_2 = False
            similar_3 = False
            similar_4 = False
            similar_5 = False
            similar_6 = False
            while not (similar_1 and similar_2 and similar_3 and similar_4 and similar_5 and similar_6):
                self.move_arm_left_joint_1(position=position)
                self.move_arm_right_joint_1(position=-1*position)
                self.move_arm_left_joint_6(position=2*position)
                self.move_arm_right_joint_6(position=2*position)
                self.move_arm_left_joint_7(position=1*position)
                self.move_arm_right_joint_7(position=1*position)
                # WARNING: THE COMMAND HAS TO BE PUBLISHED UNTIL THE GOAL IS REACHED
                # This is because, when publishing a topic, the first time doesn't always work.
                similar_1 = self.dilomo_check_continuous_joint_value(
                    joint_name="arm_left_joint_1", value=position)
                similar_2 = self.dilomo_check_continuous_joint_value(
                    joint_name="arm_right_joint_1", value=position)
                similar_3 = self.dilomo_check_continuous_joint_value(
                    joint_name="arm_left_joint_6", value=2*position)
                similar_4 = self.dilomo_check_continuous_joint_value(
                    joint_name="arm_right_joint_6", value=2*position)
                similar_5 = self.dilomo_check_continuous_joint_value(
                    joint_name="arm_left_joint_7", value=position)
                similar_6 = self.dilomo_check_continuous_joint_value(
                    joint_name="arm_right_joint_7", value=position)

                rate.sleep()

    def Close_the_fucking_door(self):

        check_rate = 5.0
        position = 1.57
        rate = rospy.Rate(check_rate)
        similar_1 = False
        while not (similar_1):
            self.move_head_joint(position=position)
            # WARNING: THE COMMAND HAS TO BE PUBLISHED UNTIL THE GOAL IS REACHED
            # This is because, when publishing a topic, the first time doesn't always work.
            similar_1 = self.dilomo_check_continuous_joint_value(
            joint_name="head_joint", value=position)
            rate.sleep()

    def movement_wave_hands(self):

        check_rate = 5.0
        position = 0.5

        rate = rospy.Rate(check_rate)
        for repetition in range(2):
            similar_1 = False
            similar_2 = False
            while not (similar_1 and similar_2):
                self.move_arm_left_joint_2(position=position)
                self.move_arm_right_joint_2(position=-position)
                # WARNING: THE COMMAND HAS TO BE PUBLISHED UNTIL THE GOAL IS REACHED
                # This is because, when publishing a topic, the first time doesn't always work.
                similar_1 = self.dilomo_check_continuous_joint_value(
                    joint_name="arm_left_joint_2", value=position)
                similar_2 = self.dilomo_check_continuous_joint_value(
                    joint_name="arm_right_joint_2", value=-position)

                rate.sleep()
            position *= -1

    def movement_sayno(self):
        """
        Make Mira say no with the head
        :return:
        """
        check_rate = 5.0
        position = 0.7

        rate = rospy.Rate(check_rate)
        for repetition in range(2):
            similar = False
            while not similar:
                self.move_head_joint(position=position)
                # WARNING: THE COMMAND HAS TO BE PUBLISHED UNTIL THE GOAL IS REACHED
                # This is because, when publishing a topic, the first time doesn't always work.
                similar = self.dilomo_check_continuous_joint_value(
                    joint_name="head_joint", value=position)

                rate.sleep()
            position *= -1

    def movement_deep_squat(self):
        """
        Make Mira say no with the head
        :return:
        """
        check_rate = 5.0
        position = 0.5

        rate = rospy.Rate(check_rate)
        for repetition in range(2):
            similar_1 = False
            similar_2 = False
            similar_3 = False
            similar_4 = False
            similar_5 = False
            similar_6 = False
            while not (similar_1 and similar_2 and similar_3 and similar_4 and similar_5 and similar_6):
                self.move_leg_left_joint_2(position=position)
                self.move_leg_right_joint_2(position=position)
                self.move_leg_left_joint_4(position=2*position)
                self.move_leg_right_joint_4(position=2*position)
                self.move_leg_left_joint_5(position=position)
                self.move_leg_right_joint_5(position=position)
                # WARNING: THE COMMAND HAS TO BE PUBLISHED UNTIL THE GOAL IS REACHED
                # This is because, when publishing a topic, the first time doesn't always work.
                similar_1 = self.dilomo_check_continuous_joint_value(
                    joint_name="leg_left_joint_2", value=position)
                similar_2 = self.dilomo_check_continuous_joint_value(
                    joint_name="leg_right_joint_2", value=position)
                similar_3 = self.dilomo_check_continuous_joint_value(
                    joint_name="leg_left_joint_4", value=2*position)
                similar_4 = self.dilomo_check_continuous_joint_value(
                    joint_name="leg_right_joint_4", value=2*position)
                similar_5 = self.dilomo_check_continuous_joint_value(
                    joint_name="leg_left_joint_5", value=position)
                similar_6 = self.dilomo_check_continuous_joint_value(
                    joint_name="leg_right_joint_5", value=position)

                rate.sleep()
            position *= -1

    def movement_bow(self):

        check_rate = 5.0
        position = -0.5
        conter = 0
        rate = rospy.Rate(check_rate)

        similar_1 = False
        similar_2 = False
        #similar_3 = False
        #similar_4 = False

        while not (similar_1 and similar_2):
         # and similar_3 and similar_4):
            conter = conter+1
            self.move_leg_left_joint_2(position=position)
            self.move_leg_right_joint_2(position=position)
            # WARNING: THE COMMAND HAS TO BE PUBLISHED UNTIL THE GOAL IS REACHED
            # This is because, when publishing a topic, the first time doesn't always work.
            similar_1 = self.dilomo_check_continuous_joint_value(
                joint_name="arm_left_joint_2", value=position)
            similar_2 = self.dilomo_check_continuous_joint_value(
                joint_name="arm_right_joint_2", value=-position)
            # if conter > 15:
            #  self.move_leg_left_joint_2(position=position)
            # self.move_leg_right_joint_2(position=position)

            if conter > 20:
                break

            rate.sleep()

    def movement_random_loop(self):
        """
        Executed movements in a random way
        :return:
        """
        rospy.loginfo("Start Moving DiLoMo...")
        while not rospy.is_shutdown():
            self.dilomo_default()
            #self.Close_the_fucking_door()
            self.movement_wave_hands()
            #self.dilomo_deform()
            #self.movement_sayno()
            #self.deform_arms()
            #self.movement_deep_squat()
           


if __name__ == "__main__":
    dilomo_jointmover_object = dilomoJointMover()
    dilomo_jointmover_object.movement_random_loop()
