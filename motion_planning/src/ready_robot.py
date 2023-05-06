#! /usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


"""
This file commands the robot to return to the "Ready" position. 
1) Move the arm to the "Ready" position
2) Move the gripper to the "Open" position
"""


planning_time = 30  # [s] Planning time for computation
planning_attempts = 300  # planning attempts

# Initialize moveit_commander module
moveit_commander.roscpp_initialize(sys.argv)
# Initialize ROS node
rospy.init_node('ready_robot', anonymous=True)
# Create a RobotCommander object, which is an interface to robot
robot = moveit_commander.RobotCommander()
# Create a PlanningSceneInterface object, which is an interface to the world around the robot
scene = moveit_commander.PlanningSceneInterface()    


# Defining a topic publisher, which will publish to the “/move_group/display_planned_path” topic
# Enable visualizing the planned motion through RViz
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)


#####################################################################################
# Step 1
print("Step 1: Moving from current pose to 'ready' pose")
# Create a MoveGroupCommander object, which is an interface to the manipulator group defined during the MoveIt package creation
# This allows us to interact with this specific set of joints
arm_group = moveit_commander.MoveGroupCommander("arm", wait_for_servers=300)

# Set planning information for arm group
arm_group.set_num_planning_attempts(planning_attempts)
arm_group.set_planning_time(planning_time)

arm_group.set_named_target("ready")   # Switch from default pose to "ready" pose
arm_execute = arm_group.go()    # Execute plan
print(f"Step 1 Status: {arm_execute}")

#####################################################################################
# Step 2
if arm_execute == True:
	print("Step 2: Moving from default gripper pose to 'open' pose")

	# Create another MoveGroupCommander object for the gripper
	gripper_group = moveit_commander.MoveGroupCommander("gripper", wait_for_servers=300)

	gripper_group.set_named_target("open")   # Switch from default pose to "grasp" pose
	gripper_execute = gripper_group.go()    # Execute plan
	print(f"Step 2 Status: {gripper_execute}")
else:
	print("Skipping Step 2: Moving from default gripper pose to 'open' pose")

#####################################################################################

rospy.sleep(2)
# Shutdown the moveit_commander module
moveit_commander.roscpp_shutdown()

