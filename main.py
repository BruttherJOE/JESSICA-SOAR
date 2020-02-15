#!/usr/bin/env python

# Wait for capsule
# Move capsule into coffee machine
# Move to cup prime
# Dispense cup
# Move cup to coffee machine
# Activate electromagnet
# Move cup to serving platform
# Drop cup
from __future__ import print_function
import time
import sys
import datetime

import rospy
import moveit_commander
import moveit_msgs
import geometry_msgs
from tf.transformations import quaternion_from_euler

import running
from niryo_one_python_api.niryo_one_api import *

group_name = 'arm'
group = moveit_commander.MoveGroupCommander(group_name)
SENSOR = GPIO_1A
MOTOR = GPIO_2A
ELECTROMAGNET = GPIO_2C

CROUCHINGTIGER=[0, 0.428, -1.237, 0.012, 0.814, -0.046]
NIRYO = NiryoOne()
NIRYO.pin_mode(SENSOR, 1)
NIRYO.pin_mode(MOTOR, 0)
NIRYO.pin_mode(ELECTROMAGNET, 0)
NIRYO.change_tool(TOOL_GRIPPER_1_ID)

# MAIN FUNCTION
def coffee_cycle(niryo_one):
    start()
    capsule_cycle(niryo_one)
    move_to_cup(niryo_one)
    dispense_cup(niryo_one)
    move_cup_to_machine(niryo_one)
    prepare_coffee(niryo_one)
    deliver_coffee(niryo_one)


def nowait_coffee_cycle(niryo_one):
    start()
    capsule_cycle(niryo_one)
    move_to_cup(niryo_one)
    dispense_cup(niryo_one)
    move_cup_to_machine(niryo_one)
    deliver_coffee(niryo_one)

# MOVEMENT FUNCTIONS
def move_joints(joint_goal):
    global group
    group.go(joint_goal, wait=True)

    while not check_joints(joint_goal):
#        move_pose((0.238, 0, 0.418), (0, 0, 0))
        try:
            start(niryo_api = True)
        except NiryoOneException as e:
            try:
                start(niryo_api = False)
            except NiryoOneException as e:
                try:
                    home(niryo_api = True)
                except NiryoOneException as e:
                    home(niryo_api = False)


        print("new move joint done")
        group.go(joint_goal, wait=True)
    return joint_goal


def move_pose(position, orientation):
    """
    position: (x, y, z)
    orientation: (x, y, z)
    """
    global group
    pose_goal = geometry_msgs.msg.Pose()

    x, y, z = position
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    r, p, y = orientation
    x, y, z, w = quaternion_from_euler(r, p, y)
    pose_goal.orientation.x = x
    pose_goal.orientation.y = y
    pose_goal.orientation.z = z
    pose_goal.orientation.w = w

    group.set_pose_target(pose_goal)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    return pose_goal


def shift_z(dz):
    # vertical movement only, by delta instead
    global group
    pose = group.get_current_pose().pose
    pose.position.z += dz

    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    return pose

