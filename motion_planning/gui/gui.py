#! /usr/bin/env python3

import rospy
import roslaunch
from kivy.lang import Builder
from kivymd.app import MDApp
import subprocess


class RobotApp(MDApp):
  def build(self):
    # Load the file that describes the appearance of the GUI
    return Builder.load_file('gui.kv')
  
  # Function that runs the launch file that starts the program when the button is pressed
  def start_program(self, start_program_btn):
    start_program_btn.disabled = True    # Disable the button after it's pressed to avoid accidentlly shutting everything down
    uuid = roslaunch.rlutil.get_or_generate_uuid(options_runid=None, options_wait_for_master=False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/joseph/catkin_ws/src/ur5e/ur5e_description/launch/ur5e.launch"], is_core=True)
    launch.start()
  
  # Function that launches file to command robot to move to "Ready" position
  def ready_robot(self, ready_robot_btn):
    subprocess.call(["/home/joseph/catkin_ws/src/ur5e/motion_planning/src/ready_robot.py"])
  
  # Function that launches file to pick & place red box
  def pick_red(self, pick_red_btn):
    subprocess.call(["/home/joseph/catkin_ws/src/ur5e/motion_planning/src/planning_execution.py", "red"])
  
  # Function that launches file to pick & place green box
  def pick_green(self, pick_green_btn):
    subprocess.call(["/home/joseph/catkin_ws/src/ur5e/motion_planning/src/planning_execution.py", "green"])
  
  # Function that launches file to pick & place orange box
  def pick_orange(self, pick_orange_btn):
    subprocess.call(["/home/joseph/catkin_ws/src/ur5e/motion_planning/src/planning_execution.py", "orange"])
  

if __name__ == '__main__':
  RobotApp().run()

