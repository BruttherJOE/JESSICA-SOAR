#!/usr/bin/env python
from __future__ import print_function

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from niryo_one_python_api.niryo_one_api import *

rospy.init_node('niryo')

group_name = 'arm'
group = moveit_commander.MoveGroupCommander(group_name)

n = NiryoOne()

waypoints = [group.get_current_pose().pose for i in range(4)]


def set_waypoints(idx, x, y, z):
    waypoints[idx].position.x = x
    waypoints[idx].position.y = y
    waypoints[idx].position.z = z

# zero then move across 4 waypoints
n.move_joints([0] * 6)
set_waypoints(0, 0.226, 0.159, 0.268)
set_waypoints(1, 0.138, 0.229, 0.243)
set_waypoints(2, 0.079, 0.242, 0.202)
set_waypoints(3, -0.02, 0.201, 0.131)

# move to last waypoint directly to test trajectory difference
# waypoints = [group.get_current_pose().pose]
# set_waypoints(0, -0.02, 0.201, 0.131)

plan, fraction = group.compute_cartesian_path(waypoints, 0.01, 0.0)
group.execute(plan, wait=True)
