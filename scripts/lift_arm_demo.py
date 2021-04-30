#!/usr/bin/env python
import time
import rospy
from math import pi, sin, cos, acos
import random
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
"""
Topics To Write on:
type: std_msgs/Float64
/lift_arm/main_arm_joint_position_controller/command
/lift_arm/upper_arm_joint_position_controller/command
/lift_arm/rack_joint_position_controller/command
"""

class ArmJointMover(object):

    def __init__(self):
        rospy.init_node('jointmover_demo', anonymous=True)
        rospy.loginfo("Lift Arm JointMover Initialising...")
        self.pub_la_main_arm_joint_position = rospy.Publisher('/lift_arm/main_arm_joint_position_controller/command',
                                                            Float64,
                                                            queue_size=1)
        self.pub_la_upper_arm_joint_position = rospy.Publisher('/lift_arm/upper_arm_joint_position_controller/command',
                                                             Float64,
                                                             queue_size=1)
        self.pub_la_rack_joint_position = rospy.Publisher('/lift_arm/rack_joint_position_controller/command',
                                                           Float64,
                                                           queue_size=1)
        joint_states_topic_name = "/lift_arm/joint_states"
        rospy.Subscriber(joint_states_topic_name, JointState, self.lift_arm_joints_callback)
        lift_arm_joints_data = None
        while lift_arm_joints_data is None:
            try:
                lift_arm_joints_data = rospy.wait_for_message(joint_states_topic_name, JointState, timeout=5)
            except:
                rospy.logwarn("Time out " + str(joint_states_topic_name))
                pass

        self.lift_arm_joint_dictionary = dict(zip(lift_arm_joints_data.name, lift_arm_joints_data.position))

    def move_lift_arm_all_joints(self, roll, pitch, yaw):
        angle_roll = Float64()
        angle_roll.data = roll
        angle_pitch = Float64()
        angle_pitch.data = pitch
        angle_yaw = Float64()
        angle_yaw.data = yaw
        self.pub_mira_roll_joint_position.publish(angle_roll)
        self.pub_mira_pitch_joint_position.publish(angle_pitch)
        self.pub_mira_yaw_joint_position.publish(angle_yaw)

    def move_mira_roll_joint(self, position):
        """
        limits radians : lower="-0.2" upper="0.2"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_mira_roll_joint_position.publish(angle)

    def move_mira_pitch_joint(self, position):
        """
        limits radians : lower="0" upper="0.44"
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_mira_pitch_joint_position.publish(angle)

    def move_mira_yaw_joint(self, position):
        """
        Limits : continuous, no limits
        :param position:
        :return:
        """
        angle = Float64()
        angle.data = position
        self.pub_mira_yaw_joint_position.publish(angle)

    def lift_arm_joints_callback(self, msg):
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
        self.lift_arm_joint_dictionary = dict(zip(msg.name, msg.position))

    def lift_arm_check_joint_value(self, joint_name, value, error=0.1):
        """
        Check the joint by name 'pitch_joint', 'roll_joint', 'yaw_joint' is near the value given
        :param value:
        :return:
        """
        similar = self.lift_arm_joint_dictionary.get(joint_name) >= (value - error ) and self.lift_arm_joint_dictionary.get(joint_name) <= (value + error )

        return similar

    def convert_angle_to_unitary(self, angle):
        """
        Removes complete revolutions from angle and converts to positive equivalent
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

    def lift_arm_check_continuous_joint_value(self, joint_name, value, error=0.1):
        """
        Check the joint by name 'pitch_joint', 'roll_joint', 'yaw_joint' is near the value given
        We have to convert the joint values removing whole revolutions and converting negative versions
        of the same angle
        :param value:
        :return:
        """
        joint_reading = self.lift_arm_joint_dictionary.get(joint_name)
        clean_joint_reading = self.convert_angle_to_unitary(angle=joint_reading)
        clean_value = self.convert_angle_to_unitary(angle=value)

        dif_angles = self.assertAlmostEqualAngles(clean_joint_reading, clean_value)
        similar = dif_angles <= error

        return similar

    def lift_arm_movement_sayno(self):
        """
        Make lift_arm say no with the head
        :return:
        """
        check_rate = 5.0
        position = 0.7


        rate = rospy.Rate(check_rate)
        for repetition in range(2):
            similar = False
            while not similar:
                self.move_mira_yaw_joint(position=position)
                # WARNING: THE COMMAND HAS TO BE PUBLISHED UNTIL THE GOAL IS REACHED
                # This is because, when publishing a topic, the first time doesn't always work.
                similar = self.mira_check_continuous_joint_value(joint_name="yaw_joint", value=position)

                rate.sleep()
            position *= -1



    def mira_movement_look(self, roll, pitch, yaw):
        """
        Make Mira look down
        :return:
        """
        check_rate = 5.0
        position_roll = roll
        position_pitch = pitch
        position_yaw = yaw

        similar_roll = False
        similar_pitch = False
        similar_yaw = False
        rate = rospy.Rate(check_rate)
        while not (similar_roll and similar_pitch and similar_yaw):
            self.move_mira_all_joints(position_roll, position_pitch, position_yaw)
            similar_roll = self.mira_check_continuous_joint_value(joint_name="roll_joint", value=position_roll)
            similar_pitch = self.mira_check_continuous_joint_value(joint_name="pitch_joint", value=position_pitch)
            similar_yaw = self.mira_check_continuous_joint_value(joint_name="yaw_joint", value=position_yaw)
            rate.sleep()


    def mira_lookup(self):

        self.mira_movement_look(roll=0.0,pitch=0.3,yaw=1.57)

    def mira_lookdown(self):

        self.mira_movement_look(roll=0.0,pitch=0.0,yaw=1.57)

    def mira_movement_laugh(self, set_rpy=False, roll=0.05, pitch=0.0, yaw=1.57, n_giggle=15):
        """
        Giggle in a given pitch yaw configuration
        :return:
        """
        position_roll = roll
        position_pitch = pitch
        position_yaw = yaw
        for repetitions in range(n_giggle):
            if set_rpy:
                self.move_mira_all_joints(position_roll, position_pitch, position_yaw)
            else:
                self.move_mira_roll_joint(position_roll)
            time.sleep(0.1)
            position_roll *= -1

    def lift_arm_moverandomly(self):
        roll = random.uniform(-0.15, 0.15)
        pitch = random.uniform(0.0, 0.3)
        yaw = random.uniform(0.0, 2*pi)
        self.lift_arm_movement_look(roll, pitch, yaw)

    def movement_random_loop(self):
        """
        Executed movements in a random way
        :return:
        """
        rospy.loginfo("Start Moving lift_arm...")
        while not rospy.is_shutdown():
            self.lift_arm_moverandomly()
            self.lift_arm_movement_laugh()


if __name__ == "__main__":
    lift_arm_jointmover_object = ArmJointMover()
    lift_arm_jointmover_object.movement_random_loop()


