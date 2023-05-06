#! /usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


# Initialize moveit_commander module
moveit_commander.roscpp_initialize(sys.argv)
# Initialize ROS node
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)


# Create a RobotCommander object, which is an interface to robot
robot = moveit_commander.RobotCommander()
# Create a PlanningSceneInterface object, which is an interface to the world around the robot
scene = moveit_commander.PlanningSceneInterface()    
# Create a MoveGroupCommander object, which is an interface to the manipulator group defined during the MoveIt package creation
# This allows us to interact with this specific set of joints
group = moveit_commander.MoveGroupCommander("arm")
# Defining a topic publisher, which will publish to the “/move_group/display_planned_path” topic
# Enable visualizing the planned motion through RViz
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

# Create a Pose object and assign values to variables
# To see the full structure of Pose message, type:
# rosmsg show geometry_msgs/Pose
pose_target = geometry_msgs.msg.Pose()
pose_target.position.x = 0.06135314469340977
pose_target.position.y = -0.6197422797608182
pose_target.position.z = 0.4436469888791478
pose_target.orientation.w = 0.36896770508633187
"""
EE Limits
x: +/-0.75
y: +/-0.8
z: 0.05, 0.95

HOME EE POSE
pose_target.position.x = 0.8155955716581806
pose_target.position.y = 0.13369226692216235
pose_target.position.z = 0.052371885144400096
pose_target.orientation.w = 0.00728548188930851

RANDOM GOAL EE POSE
pose_target.position.x = 0.06135314469340977
pose_target.position.y = -0.6197422797608182
pose_target.position.z = 0.4436469888791478
pose_target.orientation.w = 0.36896770508633187
"""

# Passing target pose to move group
#group.set_pose_target(pose_target)

# To plan to a predefined pose
group.set_named_target("start")

# Telling group to calculate plan
plan1 = group.plan()


# Keep node running until shutdown signal received
# Keep processing any events until shutdown signal received
# Same as rospy.spin, but sleep is for a specific length of time
# 5 seconds in this example
rospy.sleep(5)

# Execute the last trajectory that has been set for the Planning Group
group.go(wait=True)

# Calling `stop()` ensures that there is no residual movement
group.stop()

# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets().
group.clear_pose_targets()

# Shutdown the moveit_commander module
moveit_commander.roscpp_shutdown()
