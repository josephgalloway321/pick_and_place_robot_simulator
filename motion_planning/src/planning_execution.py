#! /usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from motion_planning.srv import pick_place_coordinates, pick_place_coordinatesResponse


"""
This file commands the robot's physical movements.

Plan for Pick & Place
1) Call the pick_place_coordinates services and pass the user color choice as an argument
   Retrieve the box and bin coordinates for the color chosen from the service program
2) Setup necessary information for MoveIt planning & adding objects to scene
3) Move arm to pick location (Not too close or gripper will hit object)

TODO: Continue debugging the following steps
4) Move arm closer (z-axis) to pick object
5) Grasp the pick object & lift arm 
6) Move arm to place location (Bin)
7) Move arm closer (z-axis) to place object (Make sure cube is released close to the floor)
8) Release pick object onto place object
9) Move back to 'ready' pose
"""

###############################################################
# Step 1
# Initialize ROS node
rospy.init_node('planning_execution')

# Get user's color choice then change to String format
user_color_choice = String()
user_color_choice.data = sys.argv[1]

# Wait for service to exist in roscore; will wait until it exists
rospy.wait_for_service("pick_place_coordinates")
service_object = rospy.ServiceProxy("pick_place_coordinates", pick_place_coordinates)    
# Store service response here
response = str(service_object(user_color_choice))

# Modify incoming message so it can be converted to a dictionary
start = '{'
end = '}'
modified_response = response[response.find(start):response.rfind(end)+1]
modified_response = modified_response.replace(" \ ", "")
modified_response = modified_response.replace(" ", "")

# Convert to a dictionary then split coordinates between pick and place locations
pick_place_coordinates_dict = eval(modified_response)
print(f"\n{pick_place_coordinates_dict}")

pick_coordinates = pick_place_coordinates_dict['pick']
place_coordinates = pick_place_coordinates_dict['place']

print(f"\nPick x = {pick_coordinates[0]}\nPick y = {pick_coordinates[1]}\nPick z = {pick_coordinates[2]}")
print(f"\nPlace x = {place_coordinates[0]}\nPlace y = {place_coordinates[1]}\nPlace z = {place_coordinates[2]}")

###############################################################
# Step 2
planning_time = 30  # [s] Planning time for computation
planning_attempts = 300  # planning attempts

# Initialize moveit_commander module
moveit_commander.roscpp_initialize(sys.argv)

# Create a RobotCommander object, which is an interface to robot
robot = moveit_commander.RobotCommander()

# Create a PlanningSceneInterface object
# This will allow the robot to interact with the objects in the world
scene = moveit_commander.PlanningSceneInterface()    


# Add objects in world to scene planner for MoveIt
# TEST
# Add box to scene (object_box)
box = CollisionObject()
box.id = 'object_box'
box.header.frame_id = robot.get_planning_frame()
box_geom = SolidPrimitive()    # Collision geometry associated with the object
box_geom.type = box_geom.BOX
box_geom.dimensions = [0.06, 0.06, 0.06]
box.primitives = [box_geom]
box_pose = Pose()    # Define where the box is located
box_pose.position.x = 0.75
box_pose.position.y = 0
box_pose.position.z = 0
box_pose.orientation.w = 1
box.primitive_poses = [box_pose]
box.operation = box.ADD
scene.add_object(box)
# TEST



# Defining a topic publisher, which will publish to the “/move_group/display_planned_path” topic
# Enable visualizing the planned motion through RViz
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

#####################################################################################
# Step 3

print("\nStep 3: Move to pick object\n")

# Create a MoveGroupCommander object, which is an interface to the manipulator group defined during the MoveIt package creation
# This allows us to interact with this specific set of joints
arm_group = moveit_commander.MoveGroupCommander("arm", wait_for_servers=300)
#arm_group.set_max_acceleration_scaling_factor()    # Scaling factor for reducing maximum joint acceleration (0,1]
arm_group.set_max_velocity_scaling_factor(0.8)    # Scaling factor for reducing maximum joint velocity (0,1]

# Set planning information for arm group
arm_group.set_num_planning_attempts(planning_attempts)
arm_group.set_planning_time(planning_time)
arm_group.allow_replanning(True)

# Create a Pose object and assign values to variables
pose_target_pick = geometry_msgs.msg.Pose()

pose_target_pick.position.x = pick_coordinates[0] + 0.025 # For error in image coordinate process
pose_target_pick.position.y = pick_coordinates[1] + 0.0 # For error in image coordinate process
pose_target_pick.position.z = pick_coordinates[2] + 0.4 # Don't get too close, may knock over
# Rotate the robot arm down 
pose_target_pick.orientation.x = -0.5
pose_target_pick.orientation.y = 0.5
pose_target_pick.orientation.z = -0.5
pose_target_pick.orientation.w = 0.5

# Passing target pose to move group
arm_group.set_pose_target(pose_target_pick)
arm_group.set_goal_position_tolerance(0.001)
arm_execute = arm_group.plan()    # Telling group to calculate plan
arm_execute = arm_group.go(wait=True)    # Execute plan
print(f"Step 3 Status: {arm_execute}")

#####################################################################################
# Step 4
"""
if arm_execute == True:
	print("\nStep 4: Get closer to pick object\n")

	# Translate only the z-axis, keep everything else the same
	pose_target_pick.position.z = 0.34
	arm_group.set_pose_target(pose_target_pick)
	#arm_execute = arm_group.plan()    # Telling group to calculate plan
	arm_execute = arm_group.go(wait=True)    # Execute plan
	print(f"Step 4 Status: {arm_execute}")
else:
	print("\nSkipping Step 4: Get closer to pick object\n")
"""
#####################################################################################
# Step 5
"""
if arm_execute == True:
	print("\nStep 5: Grasp pick object & lift arm\n")

	# Create another MoveGroupCommander object for the gripper
	gripper_group = moveit_commander.MoveGroupCommander("gripper", wait_for_servers=300)

	gripper_group.set_named_target("grasp_object")   # Switch from default pose to "grasp" pose
	gripper_execute = gripper_group.go(wait=True)    # Execute plan
	print(f"Step 5 Status (Gripper): {gripper_execute}")
	# Attach object to robot's end-effector
	arm_group.attach_object('object_box')
	
	# Lift arm
	pose_target_pick.position.z = 0.4
	arm_group.set_pose_target(pose_target_pick)
	#arm_execute = arm_group.plan()    # Telling group to calculate plan
	arm_execute = arm_group.go(wait=True)    # Execute plan
	print(f"Step 5 Status (Arm): {arm_execute}")
else:
	print("\nSkipping Step 5: Grasp pick object & lift arm\n")
"""
#####################################################################################
# Step 6 (OLD)
"""
if gripper_execute == True:
	print("\nStep 6: Lift Arm\n")
	
	pose_target_pick.position.z = pose_target_pick.position.z + 0.1
	arm_group.set_pose_target(pose_target_pick)
	arm_execute = arm_group.plan()    # Telling group to calculate plan
	arm_execute = arm_group.go(wait=True)    # Execute plan
	print(f"Step 6 Status: {arm_execute}")
else:
	print("\nSkipping Step 6: Lift Arm\n")
"""
#####################################################################################
# Step 6
"""
# Create a Pose object and assign values to variables
pose_target_place = geometry_msgs.msg.Pose()
pose_target_place.position.x = place_coordinates[0] + 0.0 # For error in image coordinate process
pose_target_place.position.y = place_coordinates[1] + 0.0 # For error in image coordinate process
pose_target_place.position.z = place_coordinates[2] + 0.35 # Don't get too close, may knock over (0.35)
# Maintain arm orientation
pose_target_.orientation.x = 0
pose_target_place.orientation.y = 0
pose_target_place.orientation.z = 0
pose_target_place.orientation.w = 1

# Passing target pose to move group
arm_group.set_pose_target(pose_target_place)
arm_execute = arm_group.plan()    # Telling group to calculate plan
arm_execute = arm_group.go()    # Execute plan
"""

#####################################################################################
# Step 7
"""
pose_target_place.position.z = 0.27
arm_group.set_pose_target(pose_target_place)
arm_execute = arm_group.plan()    # Telling group to calculate plan
arm_execute = arm_group.go()    # Execute plan
"""

#####################################################################################
# Step 8
"""
gripper_group.set_named_target("open")   # Switch from default pose to "open" pose
gripper_execute = gripper_group.go()    # Execute plan
"""

#####################################################################################
# Step 9
"""
arm_group.set_named_target("ready")   # Switch from default pose to "ready" pose
arm_execute = arm_group.go()    # Execute plan
"""


rospy.sleep(2)
# Shutdown the moveit_commander module
moveit_commander.roscpp_shutdown()


