#!/usr/bin/env python

#UPDATED as of 20/02/2020

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

CROUCHINGTIGER= [-0.12, 0.578, -1.157, 0, 0.593, 0.06]
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


def move_waypoints(waypoints):
    global group
    goal = [group.get_current_pose().pose]
    for x, y, z, roll, pitch, yaw in waypoints:
        pose = group.get_current_pose().pose

        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        x, y, z, w = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = x
        pose.orientation.y = y
        pose.orientation.z = z
        pose.orientation.w = w

        goal.append(pose)

    plan, _ = group.compute_cartesian_path(goal, 0.01, 0.0)
    group.execute(plan, wait=True)
    group.stop()
    group.clear_pose_targets()
    return goal


def home(niryo_api=False):
    if niryo_api:
        NIRYO.move_joints([-1.107, -0.1, -0.395, 0.052, 0.5, -0.015])
    else:
        move_joints([-1.107, -0.1, -0.395, 0.052, 0.5, -0.015])

def start(niryo_api=False):
    if niryo_api:
        NIRYO.move_joints([-0.12, 0.578, -1.157, 0, 0.593, 0.06])
    else:
        move_joints([-0.12, 0.578, -1.157, 0, 0.593, 0.06])

def zero():
    move_joints([0, 0, 0, 0, 0, 0])

def check_joints(joints, tolerance=0.085):
    global group
    rospy.sleep(0.01)
    actual_joints = group.get_current_joint_values()
    for i in range(6):
        if abs(joints[i] - actual_joints[i]) > tolerance:
            print('Joint {:d} failed: ({:.3f}, {:.3f})'.format(i, joints[i], actual_joints[i]))
            return False
    return True


def capsule_cycle(Niryo):
    print('Capsule')
    Niryo.open_gripper(TOOL_GRIPPER_1_ID, 1000)
    # start of sequence of serve capsule (start from zero())
    move_joints([-0.375, -0.658, 0.088, 0.029, 0.655, 0.0658])

    Niryo.close_gripper(TOOL_GRIPPER_1_ID, 1000)
    shift_z(0.1)    
    move_joints([-0.12, 0.578, -1.157, 0, 0.593, 0.06])
    move_joints([-1.533, 0.60, -0.462, 0.034, -0.127, 0.066])
    move_joints([-1.549, -0.93, 1.199, -0.066, -1.273, 0.172])

    Niryo.open_gripper(TOOL_GRIPPER_1_ID, 1000)
    move_joints([-1.533, 0.60, -0.462, 0.034, -0.127, 0.066])


def move_to_cup(Niryo):
    print('move to cup')
    home()
    move_joints([-1.115, -0.61, 0.344, 0.04, 0.28, 0.015])
    Niryo.close_gripper(TOOL_GRIPPER_1_ID, 1000)


def dispense_cup(Niryo):
    print('dispense cup')
    Niryo.digital_write(MOTOR, 0)
    time.sleep(0.2)
    Niryo.digital_write(MOTOR, 1)
    time.sleep(0.2)
    Niryo.digital_write(MOTOR, 0)


def move_cup_to_machine(Niryo):
    print('move cup to machine')
    move_joints([-1, -0.807, -0.705, 0.264, 1.6, 0.132])
    move_joints([-1.468, -0.868, -0.551, 0.237, 1.588, 0.127])


def prepare_coffee(Niryo):
    print("Electromagnet On")
    NIRYO.digital_write(ELECTROMAGNET, True)
    time.sleep(40)
    NIRYO.digital_write(ELECTROMAGNET, False)
    print("Electromagnet Off")


def deliver_coffee(Niryo):
    print('deliver coffee')
    move_joints([-1, -0.807, -0.705, 0.264, 1.6, 0.132])
    move_joints([-1.042, 0.425, -0.361, -0.014, 0.07, 0.132])
    move_joints([-0.073, -0.16, -0.267, -0.018, 0.646, 0.091])
    move_joints([-0.067, -0.326, -0.185, -0.077, 0.584, 0.081])
    Niryo.open_gripper(TOOL_GRIPPER_1_ID, 1000)
    move_joints([-0.12, 0.578, -1.157, 0, 0.593, 0.06])     #return to crouching tiger
    move_joints([-0.141, 0.443, -0.302, 0.041, 1.507, 0.051])   #start waving
    move_joints([-0.141, 0.421, -0.4, 0.565, 1.519, 0.121])
    move_joints([-0.141, 0.414, -0.393, -0.619, 1.507, 0.101])
    move_joints([-0.141, 0.421, -0.4, 0.565, 1.519, 0.121])
    move_joints([-0.141, 0.414, -0.393, -0.619, 1.507, 0.101])
    start()

def short_activate(t=0.8):
    """ 
    Quick slideover of magnet (for 1 second). Necessary to activate cleaning function
    """
    NIRYO.digital_write(ELECTROMAGNET, True)
    rospy.sleep(t)
    NIRYO.digital_write(ELECTROMAGNET, False)
    return True


def arm_script():
    latest_time = datetime.datetime.now()
    print('Start loop')
    start()
    while running.bool:
        # Takes 2 seconds of the sensor being activated
        # Checks 20 times over the next 2 seconds
        trigger_time = 2
        count = 0
        check_ticks = 20

        # Sensor output is inverted so we have to NOT all of them(?)
        trigger = not NIRYO.digital_read(SENSOR)
        if trigger:
            while count < trigger_time:
                trigger = not NIRYO.digital_read(SENSOR)
                print(str(count)+ '/' + str(trigger_time))
                time.sleep(trigger_time/float(check_ticks))
                count += trigger_time/float(check_ticks)
                if NIRYO.digital_read(SENSOR):
                    trigger = False
                    break
        time_diff = datetime.datetime.now() - latest_time
        print(str(time_diff.seconds) + " seconds since last run")
        if time_diff.seconds/60. >= 0.5:
            NIRYO.activate_learning_mode(1)
        else:
            if not check_joints(CROUCHINGTIGER):
                start()
        if time_diff.seconds/60. > 20:
            short_activate()
	    latest_time = datetime.datetime.now()
        if trigger:
            coffee_cycle(NIRYO)
            latest_time = datetime.datetime.now()


if __name__ == "__main__":
    print('Init node')
    rospy.init_node('move_group_python', anonymous=True, disable_signals=True)
    print('Init moveit')
    group_name = 'arm'
    group = moveit_commander.MoveGroupCommander(group_name)

    arg = sys.argv[1]
    latest_time = datetime.datetime.now()
    if arg == "main":
        print('Start loop')
        start()
        while True:
            # Takes 2 seconds of the sensor being activated
            # Checks 20 times over the next 2 seconds
            trigger_time = 2
            count = 0
            check_ticks = 20
    
            # Sensor output is inverted so we have to NOT all of them(?)
            trigger = not NIRYO.digital_read(SENSOR)
            if trigger:
                while count < trigger_time:
                    trigger = not NIRYO.digital_read(SENSOR)
                    print(str(count)+ '/' + str(trigger_time))
                    time.sleep(trigger_time/float(check_ticks))
                    count += trigger_time/float(check_ticks)
                    if NIRYO.digital_read(SENSOR):
                        trigger = False
                        break
            # print("FINAL TRIGGER: ", trigger)
            time_diff = datetime.datetime.now() - latest_time
            if time_diff.seconds/60. >= 1:
                NIRYO.activate_learning_mode(1)
            else:
                if not check_joints(CROUCHINGTIGER):
                    start()
            if time_diff.seconds/60. > 20:
		short_activate()
		latest_time = datetime.datetime.now()
            if trigger:
                NIRYO.activate_learning_mode(0)
                coffee_cycle(NIRYO)
                latest_time = datetime.datetime.now()
    elif arg=="capsule":
        start()
        capsule_cycle(NIRYO)
    elif arg=="coffee":
        prepare_coffee(NIRYO)
    elif arg=="deliver":
        start()
        #move_pose((0.064, -0.196, 0.077), (-0.048, 0.027, -2.101))
        deliver_coffee(NIRYO)
    elif arg == 'start':
        start()
    elif arg == 'zero':
        NIRYO.move_joints([0]*6)
        rospy.sleep(2)
    elif arg == 'boot_coffee_machine':
        short_activate()
    elif arg == 'nowait':
        for i in range(10):
            nowait_coffee_cycle(NIRYO)
    elif arg == 'dispense':
        dispense_cup(NIRYO)
    elif arg == 'calibrate':
        NIRYO.calibrate_manual()
