#! /usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


planning_time = 20  # [s] Planning time for computation
planning_attempts = 200  # planning attempts

# Initialize moveit_commander module
moveit_commander.roscpp_initialize(sys.argv)
# Initialize ROS node
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
# Create a RobotCommander object, which is an interface to robot
robot = moveit_commander.RobotCommander()
# Create a PlanningSceneInterface object, which is an interface to the world around the robot
scene = moveit_commander.PlanningSceneInterface()    


# Defining a topic publisher, which will publish to the “/move_group/display_planned_path” topic
# Enable visualizing the planned motion through RViz
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)


"""
Current Plan
1) Plan and execute from default pose to "ready" pose for arm planning group
2) Plan and execute from default to "grasp" pose for gripper planning group
3) Bring the arm close to the object (Keep distance to avoid hitting object)
4) Move closer to the object
5) Grasp object
"""
#####################################################################################
# Step 1
print("Step 1")

# Create a MoveGroupCommander object, which is an interface to the manipulator group defined during the MoveIt package creation
# This allows us to interact with this specific set of joints
arm_group = moveit_commander.MoveGroupCommander("arm", wait_for_servers=300)

# Set planning information for arm group
arm_group.set_num_planning_attempts(planning_attempts)
arm_group.set_planning_time(planning_time)

arm_group.set_named_target("ready")   # Switch from default pose to "ready" pose
arm_execute = arm_group.go()    # Execute plan

#####################################################################################
# Step 2
print("Step 2")

# Create another MoveGroupCommander object for the gripper
gripper_group = moveit_commander.MoveGroupCommander("gripper", wait_for_servers=300)

gripper_group.set_named_target("open")   # Switch from default pose to "grasp" pose
gripper_execute = gripper_group.go()    # Execute plan

#####################################################################################
"""
# Step 3
print("Step 3")

# Create a Pose object and assign values to variables
pose_target_object_box = geometry_msgs.msg.Pose()
pose_target_object_box.position.x = 0.8
pose_target_object_box.position.y = 0.0
pose_target_object_box.position.z = 0.45
pose_target_object_box.orientation.x = -0.5
pose_target_object_box.orientation.y = 0.5
pose_target_object_box.orientation.z = -0.5
pose_target_object_box.orientation.w = 0.5

# Passing target pose to move group
arm_group.set_pose_target(pose_target_object_box)
arm_execute = arm_group.plan()    # Telling group to calculate plan
arm_execute = arm_group.go()    # Execute plan

#####################################################################################
# Step 4
print("Step 4")
pose_target_object_box.position.z = 0.32
arm_group.set_pose_target(pose_target_object_box)
arm_execute = arm_group.plan()    # Telling group to calculate plan
arm_execute = arm_group.go()    # Execute plan

#####################################################################################
# Step 5
print("Step 5")
gripper_group.set_named_target("grasp_object")   # Switch from default pose to "grasp" pose
gripper_execute = gripper_group.go()    # Execute plan

#####################################################################################
"""
"""
# Create a Pose object and assign values to variables
pose_target = geometry_msgs.msg.Pose()
pose_target.position.x = 0.06135314469340977
pose_target.position.y = -0.6197422797608182
pose_target.position.z = 0.4436469888791478
pose_target.orientation.w = 0.36896770508633187

EE Limits
x: +/-0.75
y: +/-0.8
z: 0.05, 0.95


# Passing target pose to move group
#group.set_pose_target(pose_target)

# Telling group to calculate plan
plan1 = group.plan()


# Keep node running until shutdown signal received
# Keep processing any events until shutdown signal received
# Same as rospy.spin, but sleep is for a specific length of time
# 5 seconds for calculating plan in this example
rospy.sleep(5)

# Execute the last trajectory that has been set for the Planning Group
group.go(wait=True)

# Calling `stop()` ensures that there is no residual movement
group.stop()

# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets().
group.clear_pose_targets()
"""

rospy.sleep(5)
# Shutdown the moveit_commander module
moveit_commander.roscpp_shutdown()
