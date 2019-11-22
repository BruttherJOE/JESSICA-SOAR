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

from niryo_one_python_api.niryo_one_api import *

# MoveIt stack is used because Niryo API lags in response. Even if Niryo API is used, 
# the way it works is that it passes the values to MoveIt stack to control the arm. So might as well se MoveIt stack straight.
print('Init node')
rospy.init_node('move_group_python', anonymous=True, disable_signals=True)
print('Init moveit')
group_name = 'arm'
group = moveit_commander.MoveGroupCommander(group_name)

SENSOR = GPIO_1A  # sensor to sense capsule
MOTOR = GPIO_2A  # stepper motor which dispenses paper cups
ELECTROMAGNET = GPIO_2B  # electromagnet is used to control the coffee machine. When the electromagnet is on, the coffee machine is signaled to make coffee.
TOOL = TOOL_GRIPPER_1_ID

CROUCHINGTIGER = [0, 0.428, -1.237, 0.012, 0.814, -0.046]  # home position

niryo = NiryoOne()
niryo.pin_mode(SENSOR, PIN_MODE_INPUT)
niryo.pin_mode(MOTOR, PIN_MODE_OUTPUT)
niryo.pin_mode(ELECTROMAGNET, PIN_MODE_OUTPUT)
niryo.change_tool(TOOL)


# MAIN FUNCTION
def coffee_cycle(niryo):				# main function
    start()
    capsule_cycle(niryo)
    move_to_cup(niryo)
    dispense_cup(niryo)
    move_cup_to_machine(niryo)
    prepare_coffee(niryo)
    deliver_coffee(niryo)


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

# Move to target position
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


# shift pose in z direction
def shift_z(dz):
    # vertical movement only, by delta instead
    global group
    pose = group.get_current_pose().pose #get current pose
    pose.position.z += dz #change z position

    group.set_pose_target(pose) #set the target pose
    group.go(wait=True) #move to the position. 'wait=True' means that it will execute the next line only when the current line is completed
    group.stop() 
    group.clear_pose_targets() #clear the object - recommended
    return pose


# path planning (advised to not use it because the path planning might result in paths which breaks the hardware itself
def move_waypoints(waypoints):
    global group
    # ref: http://docs.ros.org/hydro/api/pr2_moveit_tutorials/html/planning/scripts/doc/move_group_python_interface_tutorial.html#cartesian-paths
    # i guess send waypoints as a list of (x, y, z, roll, pitch, yaw) tuples?
    goal = [group.get_current_pose().pose]
    for x, y, z, roll, pitch, yaw in waypoints:
        pose = group.get_current_pose().pose #create new object to change value

        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        x, y, z, w = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = x
        pose.orientation.y = y
        pose.orientation.z = z
        pose.orientation.w = w

        goal.append(pose) #update goal

    plan, _ = group.compute_cartesian_path(goal, 0.01, 0.0)
    group.execute(plan, wait=True)
    group.stop()
    group.clear_pose_targets()
    return goal


def home(niryo_api=False):	# move to home position through NIRYO or move_it
	''' home is a waypoint in between moving from the cup to the coffee machine '''
    if niryo_api:
        NIRYO.move_joints([0, 0, -1.162, 0.144, 1.157, 0])
    else:
        move_joints([0, 0, -1.162, 0.144, 1.157, 0])

def start(niryo_api=False):	# move from calibration position to crouching tiger position
	''' 
	the arm is supposed to auto correct till it reaches the position, if running through NIRYO fails,
	run though move_it instead
	'''
    if niryo_api:		# move using NIRYO to position
        NIRYO.move_joints(CROUCHINGTIGER)
    else:			# move using move_it to position - both positions are the same
        move_joints(CROUCHINGTIGER)


# Return to zero positions for all joints
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


# check if the difference between the current positions of the joints and the desired position is within the tolerance
def capsule_cycle(niryo):	# Picking up the capsule and delivering it
    print('Capsule')
    niryo.open_gripper(TOOL, 1000) 		                # open the gripper, 1000 is the speed of the motor
    # start of sequence of serve capsule (start from zero())
    move_joints([-0.246, -0.711, 0.013, 0.260, 0.645, -0.177])	# move to capsule pickup point

    niryo.close_gripper(TOOL, 1000)	            	        # close. Pick up capsule
    shift_z(0.1)						# takes current location, moves in the z direction by 10cm
    move_joints([-1.516, 0.102, -0.265, -0.025, -1.069, -0.020])# waypoint
    move_joints([-1.536, -0.843, 0.881, 0.005, -1.460, -0.015])	# move to drop-off position
    
    niryo.open_gripper(TOOL, 1000)	            		# open gripper
    move_joints([-1.555, -0.630, 0.952, 0.020, -1.476, -0.015]) # move to... ???

def move_to_cup(niryo):						# move to pick up cup
    print('move to cup')
    home()							# pass through home waypoint
    move_joints([1.490, -0.368, -0.659, -0.252, 1.146, 0.056])	# move to pick up position
    niryo.close_gripper(TOOL, 1000)		                # close gripper and wait for cup to dispense


def dispense_cup(niryo):					# dispense cup
    print('dispense cup')
    niryo.digital_write(MOTOR, 1)				# turn on motor, 1: High, 0: Low
    time.sleep(0.2)						
    niryo.digital_write(MOTOR, 0)				# turn off


def move_cup_to_machine(niryo):					# move cup to coffee machine
    print('move cup to machine')
    move_joints([0, 0, -1.162, 0.144, 1.157, 0])		# waypoint
    move_joints([-1.199, -0.82, -0.795, 0.828, 1.57, -0.01])	# final coffee pick up position


def prepare_coffee(niryo):					# prepare coffee
    print("Electromagnet On")					
    niryo.digital_write(ELECTROMAGNET, 1)			# NIRYO sends HIGH to Arduino stepper_control code, 
								# Arduino code flips and sends LOW to Relay, Relay switches on Electromagnet
    time.sleep(60)						# wait 60 seconds for coffee
    niryo.digital_write(ELECTROMAGNET, 0)			# Switch off Electromagnet
    print("Electromagnet Off")


def deliver_coffee(niryo):					# deliver coffee
    print('deliver coffee')
    move_joints([-0.637, -0.769, -0.913, 0.622, 1.634, 0.03])
    move_joints([-0.685, -0.080, -0.466, 0.637, 0.577, -0.633])
    move_joints([0.047, -0.184, -0.313, 0.0, 0.525, -.046])
    move_joints([0.060, -0.445, -0.298, 0.103, 0.733, -0.121])
    niryo.open_gripper(TOOL, 1000)
    start()

def short_activate(niryo, t=0.8):
    """ 
    Quick slideover of magnet (for 1 second). Necessary to activate cleaning function
    """
    niryo.digital_write(ELECTROMAGNET, True)
    rospy.sleep(t)
    niryo.digital_write(ELECTROMAGNET, False)
    return True


if __name__ == "__main__":
    arg = sys.argv[1]
    if arg == "main":
        print('Start loop')        
        latest_time = datetime.datetime.now()
        start()
        while True:
            # Takes 2 seconds of the sensor being activated
            # Checks 20 times over the next 2 seconds
            trigger_time = 2.0
            count = 0
            check_ticks = 20
    
            # Sensor output is inverted so we have to NOT all of them(?)
            trigger = not niryo.digital_read(SENSOR)
            if trigger:
                while count < trigger_time:
                    trigger = not niryo.digital_read(SENSOR)
                    time.sleep(trigger_time / check_ticks)
                    
                    count += trigger_time / check_ticks
                    print('%.1f / %.1f' % (count, trigger_time))

                    if niryo.digital_read(SENSOR):
                        trigger = False
                        break
            
            time_diff = datetime.datetime.now() - latest_time

            if time_diff.seconds / 60. >= 0.5:  # learning mode after 0.5 mins idle
                niryo.activate_learning_mode(True)
            else:  # otherwise maintain crouching tiger
                if not check_joints(CROUCHINGTIGER):
                    start()

            if time_diff.seconds / 60. > 20:  # prevent coffee machine from sleeping
		short_activate()
		latest_time = datetime.datetime.now()

            if trigger:
                niryo.activate_learning_mode(False)
                coffee_cycle(niryo)
                latest_time = datetime.datetime.now()
