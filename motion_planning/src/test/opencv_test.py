#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

print("Hello!")

# Initialize the ROS node (opemcv_example), and allow multiple nodes to be run with this name
rospy.init_node('opencv_example', anonymous=True)

# Initialize the CvBridge class
bridge = CvBridge()

def image_callback(img_msg):
  # Try to convert the ROS Image message to a CV2 Image
  try:
    #cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
  except CvBridgeError as e:
    print("Error!")
    rospy.logerr("CvBridge Error: {0}".format(e))
  
  # Show the captured image
  cv2.imshow("Image Window", cv_image)
  cv2.waitKey(0)

# Initialize a subscriber to the "/camera_rgbd/rgb/image_raw" topic with the image_callback as a callback
sub_image = rospy.Subscriber("/camera_rgbd/rgb/image_raw", Image, image_callback)

# Initialize an OpenCV window name "Image Window"
#cv2.namedWindow("Image Window", 1)

# Loop to keep the program from shutting down unless ROS is shut down
while not rospy.is_shutdown():
  cv2.destroyAllWindows()
  rospy.spin()
  

