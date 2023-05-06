#! /usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


"""
This program is executed while debugging. 
It's a useful program that prints specific information about the current robot.
"""

# Initialize moveit_commander module
moveit_commander.roscpp_initialize(sys.argv)
# Initialize ROS node
rospy.init_node('robot_data', anonymous=True)


# Create a RobotCommander object, which is an interface to robot
robot = moveit_commander.RobotCommander()
# Create a PlanningSceneInterface object, which is an interface to the world around the robot
scene = moveit_commander.PlanningSceneInterface()    
# Create a MoveGroupCommander object, which is an interface to the manipulator group defined during the MoveIt package creation
# This allows us to interact with this specific set of joints
group = moveit_commander.MoveGroupCommander("arm")


# Print the following information about the robot
# Reference frame for group
print("Reference frame: %s" % group.get_planning_frame())
# End-Effector Link
print("\nEnd effector: %s" % group.get_end_effector_link())
# List of all groups for robot
print("\nRobot Groups: ")
print(robot.get_group_names())
# Current values of the joints
print("\nCurrent Joint Values: ")
print(group.get_current_joint_values())
# Current Pose of EE
print("\nCurrent Pose: ")
print(group.get_current_pose())
# General Status
print("\nRobot State: ")
print(robot.get_current_state())

# Shutdown the moveit_commander module
moveit_commander.roscpp_shutdown()

