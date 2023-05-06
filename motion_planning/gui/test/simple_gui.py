#! /usr/bin/env python3

import rospy
import roslaunch
from kivy.lang import Builder
from kivymd.app import MDApp


class TutorialApp(MDApp):
  def build(self):
    self.theme_cls.theme_syle = "Dark"
    self.theme_cls.primary_palette = "BlueGray"
    return Builder.load_file('simple_gui.kv')
  
  def start_program(self, start_program_btn):
    disabled = True
    uuid = roslaunch.rlutil.get_or_generate_uuid(options_runid=None, options_wait_for_master=False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/joseph/catkin_ws/src/ur5e/ur5e_description/launch/ur5e.launch"], is_core=True)
    launch.start()
    
  def ready_robot(self, ready_robot_btn):
    pass
  

if __name__ == '__main__':
  TutorialApp().run()
