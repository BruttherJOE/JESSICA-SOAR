#!/usr/bin/env python
import rospy
import moveit_commander
import moveit_msgs
import geometry_msgs
from tf.transformations import quaternion_from_euler

rospy.init_node('niryo_one_example')

group_name = 'arm'
group = moveit_commander.MoveGroupCommander(group_name)


def move_joints(joint_goal):
    """
    joint_goal: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
    """
    global group
    group.go(joint_goal, wait=True)
    group.stop()


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

