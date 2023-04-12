#! /usr/bin/env python
"""A helper program to test cartesian goals for the JACO and MICO arms."""

import roslib; roslib.load_manifest('kinova_demo')
import rospy

import sys
import numpy as np

import actionlib
import kinova_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose


import math
import argparse

""" Global variable """
arm_joint_number = 0
finger_number = 0
prefix = "j2n6s300" + "_"
finger_maxDist = 18.9/2/1000  # max distance for one finger
finger_maxTurn = 6800  # max thread rotation for one finger
currentCartesianCommand = [0.212322831154, -0.257197618484, 0.509646713734, 1.63771402836, 1.11316478252, 0.134094119072] # default home in unit mq


def cartesian_pose_client(position, orientation):
    """Send a cartesian goal to the action server."""
    action_address = '/' + prefix + 'driver/pose_action/tool_pose'
    client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.ArmPoseGoal()
    goal.pose.header = std_msgs.msg.Header(frame_id=(prefix + 'link_base'))
    goal.pose.pose.position = geometry_msgs.msg.Point(
        x=position.x, y=position.y, z=position.z)
    goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
        x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w)

    # print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug

    client.send_goal(goal)

    if client.wait_for_result(rospy.Duration(10.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        print('        the cartesian action timed-out')
        return None


def QuaternionNorm(Q_raw):
    qx_temp,qy_temp,qz_temp,qw_temp = Q_raw[0:4]
    qnorm = math.sqrt(qx_temp*qx_temp + qy_temp*qy_temp + qz_temp*qz_temp + qw_temp*qw_temp)
    qx_ = qx_temp/qnorm
    qy_ = qy_temp/qnorm
    qz_ = qz_temp/qnorm
    qw_ = qw_temp/qnorm
    Q_normed_ = [qx_, qy_, qz_, qw_]
    return Q_normed_


def Quaternion2EulerXYZ(Q_raw):
    Q_normed = QuaternionNorm(Q_raw)
    qx_ = Q_normed[0]
    qy_ = Q_normed[1]
    qz_ = Q_normed[2]
    qw_ = Q_normed[3]

    tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_), (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_))
    ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_)
    tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_), (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_))
    EulerXYZ_ = [tx_,ty_,tz_]
    return EulerXYZ_


def EulerXYZ2Quaternion(EulerXYZ_):
    tx_, ty_, tz_ = EulerXYZ_[0:3]
    sx = math.sin(0.5 * tx_)
    cx = math.cos(0.5 * tx_)
    sy = math.sin(0.5 * ty_)
    cy = math.cos(0.5 * ty_)
    sz = math.sin(0.5 * tz_)
    cz = math.cos(0.5 * tz_)

    qx_ = sx * cy * cz + cx * sy * sz
    qy_ = -sx * cy * sz + cx * sy * cz
    qz_ = sx * sy * cz + cx * cy * sz
    qw_ = -sx * sy * sz + cx * cy * cz

    Q_ = [qx_, qy_, qz_, qw_]
    return Q_



def getcurrentCartesianCommand(prefix_):
    # wait to get current position
    topic_address = '/' + prefix_ + 'driver/out/cartesian_command'
    print(topic_address)
    rospy.Subscriber(topic_address, kinova_msgs.msg.KinovaPose, setcurrentCartesianCommand)
    rospy.wait_for_message(topic_address, kinova_msgs.msg.KinovaPose)
    print ('position listener obtained message for Cartesian pose. ')


def setcurrentCartesianCommand(feedback):
    global currentCartesianCommand

    currentCartesianCommand_str_list = str(feedback).split("\n")

    for index in range(0,len(currentCartesianCommand_str_list)):
        temp_str=currentCartesianCommand_str_list[index].split(": ")
        currentCartesianCommand[index] = float(temp_str[1])
    # the following directly reading only read once and didn't update the value.
    # currentCartesianCommand = [feedback.X, feedback.Y, feedback.Z, feedback.ThetaX, feedback.ThetaY, feedback.Z] 
    # print 'currentCartesianCommand in setcurrentCartesianCommand is: ', currentCartesianCommand


def argumentParser(argument_):
    """ Argument parser """
    parser = argparse.ArgumentParser(description='Drive robot end-effector to command Cartesian pose')
    parser.add_argument('kinova_robotType', metavar='kinova_robotType', type=str, default='j2n6a300',
                        help='kinova_RobotType is in format of: [{j|m|r|c}{1|2}{s|n}{4|6|7}{s|a}{2|3}{0}{0}]. eg: j2n6a300 refers to jaco v2 6DOF assistive 3fingers. Please be noted that not all options are valided for different robot types.')
    parser.add_argument('unit', metavar='unit', type=str, nargs='?', default='mq',
                        choices={'mq', 'mdeg', 'mrad'},
                        help='Unit of Cartesian pose command, in mq(Position meter, Orientation Quaternion),  mdeg(Position meter, Orientation Euler-XYZ in degree), mrad(Position meter, Orientation Euler-XYZ in radian)]')
    parser.add_argument('pose_value', nargs='*', type=float, help='Cartesian pose values: first three values for position, and last three(unit mdeg or mrad)/four(unit mq) for Orientation')
    parser.add_argument('-r', '--relative', action='store_true',
                        help='the input values are relative values to current position.')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='display Cartesian pose values in alternative convention(mq, mdeg or mrad)')
    # parser.add_argument('-f', action='store_true', help='assign finger values from a file')

    args_ = parser.parse_args(argument_)
    # print('pose_mq in argumentParser 1: {}'.format(args_.pose_value))  # debug
    return args_


def kinova_robotTypeParser(kinova_robotType_):
    """ Argument kinova_robotType """
    global robot_category, robot_category_version, wrist_type, arm_joint_number, robot_mode, finger_number, prefix, finger_maxDist, finger_maxTurn 
    robot_category = kinova_robotType_[0]
    robot_category_version = int(kinova_robotType_[1])
    wrist_type = kinova_robotType_[2]
    arm_joint_number = int(kinova_robotType_[3])
    robot_mode = kinova_robotType_[4]
    finger_number = int(kinova_robotType_[5])
    prefix = "j2n6s300" + "_"
    finger_maxDist = 18.9/2/1000  # max distance for one finger in meter
    finger_maxTurn = 6800  # max thread turn for one finger



def verboseParser(verbose, position, orientation):
    """ Argument verbose """
    
    if verbose: 
        orientation_rad = Quaternion2EulerXYZ(orientation)
        orientation_deg = list(map(math.degrees, orientation_rad))
        print('Cartesian position is: {}'.format(position))
        print('Cartesian orientation in Quaternion is: ')
        print('qx {:0.3f}, qy {:0.3f}, qz {:0.3f}, qw {:0.3f}'.format(orientation.x, orientation.y, orientation.z, orientation.w))
        print('Cartesian orientation in Euler-XYZ(radian) is: ')
        print('tx {:0.3f}, ty {:0.3f}, tz {:0.3f}'.format(orientation_rad[0], orientation_rad[1], orientation_rad[2]))
        print('Cartesian orientation in Euler-XYZ(degree) is: ')
        print('tx {:3.1f}, ty {:3.1f}, tz {:3.1f}'.format(orientation_deg[0], orientation_deg[1], orientation_deg[2]))



def message_rec_callback(pose):
    #data will be a Pose message (defined to be quaternion)

    print("recieved")
    getcurrentCartesianCommand(prefix)

    #pose_mq, pose_mdeg, pose_mrad = unitParser(data)

    try:
        result = cartesian_pose_client(pose.position, pose.orientation)
        print(pose)
        print('Cartesian pose sent!')

    except rospy.ROSInterruptException:
        print ("program interrupted before completion")


    #verboseParser(args.verbose, poses)


def listener():
    rospy.init_node(prefix + 'pose_action_client')
    rospy.Subscriber("/desired_arm_pose", Pose, message_rec_callback, queue_size=1)
    rospy.spin()





if __name__ == '__main__':

    listener()
    #want to wait until we receive a message on the /robot_desired_pose topic


    
