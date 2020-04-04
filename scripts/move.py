#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print "============ Planning frame: %s" % planning_frame

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print "============ End effector link: %s" % eef_link

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print "============ Available Planning Groups:", robot.get_group_names()

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print "============ Printing robot state"
print robot.get_current_state()
print ""

# We can get the joint values from the group and adjust some of the values:
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 1.6414
joint_goal[1] = -2.6409
joint_goal[2] = -1.266
joint_goal[3] = -2.141
joint_goal[4] = 1.26761
joint_goal[5] = -3.1319

# [1.64148236827688, -2.6409354380756938, -1.2666782504505472, -2.141842478311024, 1.2676117569888241, -3.13196463459661]
# 0.00016729636728829433, 0.008896818884219115, -0.0008821595542132243, 0.0004086089412451699, 5.9098116160782865e-06, 4.022260266989264e-05]

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
move_group.go(joint_goal, wait=True)

# Calling ``stop()`` ensures that there is no residual movement
move_group.stop()

# pose_goal = geometry_msgs.msg.Pose()
# pose_goal.orientation.w = 1.0
# pose_goal.position.x = 0.4
# pose_goal.position.y = 0.1
# pose_goal.position.z = 0.4

# move_group.set_pose_target(pose_goal)

# plan = move_group.go(wait=True)
# # Calling `stop()` ensures that there is no residual movement
# move_group.stop()
# # It is always good to clear your targets after planning with poses.
# # Note: there is no equivalent function for clear_joint_value_targets()
# move_group.clear_pose_targets()

print("DONE!")