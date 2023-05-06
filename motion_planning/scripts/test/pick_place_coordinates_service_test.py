#! /usr/bin/env python3

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import imutils
import rospy

from motion_planning.srv import pick_place_coordinates, pick_place_coordinatesResponse



def callback(request):
  return pick_place_coordinatesResponse(request.color)


def merp():
  rospy.init_node("pick_place_coordinates_service")
  service = rospy.Service("pick_place_coordinates", pick_place_coordinates, callback)
  rospy.spin()


if __name__ == '__main__':
  merp()
