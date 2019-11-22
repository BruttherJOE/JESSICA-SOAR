#!/usr/bin/env python
# Docs: https://github.com/NiryoRobotics/niryo_one_ros/tree/master/niryo_one_python_api
import rospy
from niryo_one_python_api.niryo_one_api import *

rospy.init_node('niryo_one_example')

tool = TOOL_GRIPPER_1_ID
n = NiryoOne()
n.change_tool(tool)


def home():
    n.move_joints([0, 0.428, -1.237, 0.012, 0.814, -0.046])


def zero():
    # don't run this, the arm will hit something
    n.move_pose(0, 0, 0, 0, 0, 0)

def toggle_gripper(speed=300):
    # speed should be [0, 1000]
    n.open_gripper(tool, speed)
    n.close_gripper(tool, speed)
    
