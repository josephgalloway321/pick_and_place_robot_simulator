#! /usr/bin/env python3

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
import ros_numpy
import tf
import cv2
import numpy as np
import imutils
import rospy
from motion_planning.srv import pick_place_coordinates, pick_place_coordinatesResponse

"""
This script is a service that receives a color (String()) as an input and outputs a String() of coordinates.
1) When the service is called, determine which color the user has chosen
2) Capture raw image of scene then convert to CV2 image
3) Filter everything in image except chosen color
4) Convert to grayscale, blur, and threshold image
5) Create contours and calculate center of each contour
6) Use transform to convert from camera coordinate frame to world coordinate frame

"""

# Function to run when the service is called
def callback(request):
  ###############################################################
  # Step 1
  # Determine the color the user chose, match variable type to incoming message (String())
  option_1 = String()
  option_1.data = "red"
  option_2 = String()
  option_2.data = "green"
  option_3 = String()
  option_3.data = "orange"
 
  if request.color == option_1:
    choice = 1
  elif request.color == option_2:
    choice = 2
  elif request.color == option_3:
    choice = 3
  else:
    # Default ro red if no choice or invalid choice given
    choice = 1
    
  ###############################################################
  # Step 2
  # Initialize the CvBridge class, used to convert ROS image to CV2 image
  bridge = CvBridge()
  
  # Capture raw image of scene
  raw_img = rospy.wait_for_message('/camera_rgbd/rgb/image_raw', Image, timeout=1)
  
  # Try to convert the ROS Image message to a CV2 Image
  try:
    cv_image = bridge.imgmsg_to_cv2(raw_img, "bgr8")
  except CvBridgeError as e:
    print("Error!")
    rospy.logerr("CvBridge Error: {0}".format(e))
  
  ###############################################################
  # Step 3
  # Filter out everything except the pick object and place object
  # Convert the BGR color space to HSV color space
  hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
  
  # Set threshold of color in HSV space
  lower_green = np.array([50, 100, 100])
  upper_green = np.array([70, 255, 255])
  lower_red = np.array([0, 100, 20])
  upper_red = np.array([10, 255, 255])
  lower_orange = np.array([5, 50, 50])
  upper_orange = np.array([15, 255, 255])
  
  # Prepare the mask to overlay
  if choice == 1:
    mask = cv2.inRange(hsv, lower_red, upper_red)
  elif choice == 2:
    mask = cv2.inRange(hsv, lower_green, upper_green)
  elif choice == 3:
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
  
  # The black region in the mask has the value of 0, so when
  # multiplied with the original image removes all non-color regions
  result = cv2.bitwise_and(cv_image, cv_image, mask=mask)
  
  # Show masked image
  #cv2.imshow('mask', result)
  #cv2.waitKey(0)
  
  ###############################################################
  # Step 4
  # Convert to grayscale
  gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
  
  # Show gray image
  #cv2.imshow('gray', gray)
  #cv2.waitKey(0)
  
  # Blurring to reduce high frequency noise to make contour more accurate
  blurred = cv2.GaussianBlur(gray, (5, 5), 0)
  # Binarization of image by thresholding
  thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY)[1]
  
  # Show thresh image
  #cv2.imshow('thresh', thresh)
  #cv2.waitKey(0)
  ###############################################################
  # Step 5
  # Find the contours of each object in the threshold image
  cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  cnts = imutils.grab_contours(cnts)
  
  image_coordinates_array = []
  # Loop over the contours
  for c in cnts:
    # Compute the center of the contour
    M = cv2.moments(c)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    image_coordinates_array.append((cY,cX))
  
    # Draw contours and centers on masked image
    cv2.drawContours(cv_image, [c], -1, (255, 255, 255), 2)
    cv2.circle(cv_image, (cX, cY), 2, (255, 255, 255), -1)
  
  # Show contour image
  #cv2.imshow('contour', cv_image)
  #cv2.waitKey(0)
  ###########################################################
  # Step 6
  # Convert image coordinates to world coordinates
  # Get depth sensor information
  depth_points = rospy.wait_for_message('/camera_rgbd/depth/points', PointCloud2, timeout=1)
  # Convert sensor data from PointCloud2 to a ros_numpy array
  xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(depth_points, remove_nans=False)
  
  # Separate pick and place coordinates
  place_image_coordinate_x = image_coordinates_array[0][0]
  place_image_coordinate_y = image_coordinates_array[0][1]
  pick_image_coordinate_x = image_coordinates_array[1][0]
  pick_image_coordinate_y = image_coordinates_array[1][1]
  
  pick = xyz_array[pick_image_coordinate_x, pick_image_coordinate_y]
  place = xyz_array[place_image_coordinate_x, place_image_coordinate_y]
  pick_x, pick_y, pick_z = pick[0], pick[1], pick[2]
  place_x, place_y, place_z = place[0], place[1], place[2]
  
  # Transform from camera coordinate frame to world coordinate frame
  listener = tf.TransformListener()
  listener.waitForTransform("camera_rgbd_camera_depth_optical_frame", "world", rospy.Time(0), rospy.Duration(2.0))
  
  depth_point_pick = PointStamped()
  depth_point_pick.header.frame_id = "camera_rgbd_camera_depth_optical_frame"
  depth_point_pick.header.stamp = rospy.Time(0)
  depth_point_pick.point.x = pick_x
  depth_point_pick.point.y = pick_y
  depth_point_pick.point.z = pick_z
  world_coordinates_pick = listener.transformPoint("world", depth_point_pick)
  
  depth_point_place = PointStamped()
  depth_point_place.header.frame_id = "camera_rgbd_camera_depth_optical_frame"
  depth_point_place.header.stamp = rospy.Time(0)
  depth_point_place.point.x = place_x
  depth_point_place.point.y = place_y
  depth_point_place.point.z = place_z
  world_coordinates_place = listener.transformPoint("world", depth_point_place)
  
  # Put world coordinates of each object into a dictionary
  world_coordinates_dict = \
  {"pick":(world_coordinates_pick.point.x, world_coordinates_pick.point.y, world_coordinates_pick.point.z), \
   "place":(world_coordinates_place.point.x, world_coordinates_place.point.y, world_coordinates_place.point.z)}
  
  # Put dictionary in String() format to send back to robot
  world_coordinates_msg = String()
  world_coordinates_msg.data = str(world_coordinates_dict)
  
  return pick_place_coordinatesResponse(world_coordinates_msg)


def call_service():
  # Initialize the ROS node
  rospy.init_node("pick_place_coordinates_service")
  
  # Run callback function when service is called
  service = rospy.Service("pick_place_coordinates", pick_place_coordinates, callback)
  rospy.spin()


if __name__ == '__main__':
  call_service()

