#! /usr/bin/env python3

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import imutils
import rospy


"""
  Return image coordinates of the pick object and the place object using the following steps:
  1) 
  2)
  3) 
  4) 
"""

# Initialize the ROS node
rospy.init_node('get_pick_place_image_coordinates')

# Initialize the CvBridge class
bridge = CvBridge()


def image_callback(img_msg):
  
  """
  Return coordinates of the pick object and the place object in the 2D image:
  1) 
  2)
  3) 
  4) 
  """
  
  # Try to convert the ROS Image message to a CV2 Image
  try:
    cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
  except CvBridgeError as e:
    print("Error!")
    rospy.logerr("CvBridge Error: {0}".format(e))
  
  ###############################################################
  # Filter out everything except the pick object and place object
  
  # Convert the BGR color space to HSV color space
  hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
  
  # Set threshold of color in HSV space
  #lower_green = np.array([50, 100, 100])
  #upper_green = np.array([70, 255, 255])
  lower_red = np.array([0, 100, 20])
  upper_red = np.array([10, 255, 255])
  #lower_orange = np.array([5, 50, 50])
  #upper_orange = np.array([15, 255, 255])
  
  # Prepare the mask to overlay
  #mask = cv2.inRange(hsv, lower_green, upper_green)
  mask = cv2.inRange(hsv, lower_red, upper_red)
  #mask = cv2.inRange(hsv, lower_orange, upper_orange)
  
  # The black region in the mask has the value of 0, so when
  # multiplied with the original image removes all non-color regions
  result = cv2.bitwise_and(cv_image, cv_image, mask=mask)
  
  ###############################################################
  # Calculate the center of the pick object and the place object
  
  # Convert to grayscale
  gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
  # Blurring to reduce high frequency noise to make contour more accurate
  blurred = cv2.GaussianBlur(gray, (5, 5), 0)
  # Binarization of image by thresholding
  thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY)[1]
  
  # Find the contours of each object in the threshold image
  cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  cnts = imutils.grab_contours(cnts)
  
  
  image_coordinates = []
  # Loop over the contours
  for c in cnts:
    # Compute the center of the contour
    M = cv2.moments(c)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    image_coordinates.append((cY,cX))
  
  image_coordinates = str(image_coordinates)
  
  pub = rospy.Publisher("/pick_place_image_coordinates", String, queue_size=1)
  pub.publish(image_coordinates)
  
  #print(result.shape[0])
  #print(result.shape[1])  
  cv2.imshow("Result", result)
  cv2.waitKey(0)
  #TODO: Change x to u and y to v to align with image pixel coordinate convention
  

# Initialize a subscriber to the "/camera_rgbd/rgb/image_raw" topic with the image_callback as a callback
sub_image = rospy.Subscriber("/camera_rgbd/rgb/image_raw", Image, image_callback, queue_size=1)


# Loop to keep the program from shutting down unless ROS is shut down
while not rospy.is_shutdown():
  cv2.destroyAllWindows()
  rospy.spin()
