#! /usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
import ros_numpy
import tf



rospy.init_node('depth_points_test', anonymous=True)


def callback(msg):
  #print(msg.header)
  xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=False)
  #pick = xyz_array[232, 319]
  place = xyz_array[294, 456]
  #x, y, z = pick[0], pick[1], pick[2]
  x, y, z = place[0], place[1], place[2]
  print(f"x: {x}, y: {y}, z: {z}")
  
  listener = tf.TransformListener()
  listener.waitForTransform("camera_rgbd_camera_depth_optical_frame", "world", rospy.Time(0), rospy.Duration(2.0))
  depth_point = PointStamped()
  depth_point.header.frame_id = "camera_rgbd_camera_depth_optical_frame"
  depth_point.header.stamp = rospy.Time(0)
  depth_point.point.x = x
  depth_point.point.y = y
  depth_point.point.z = z
  p = listener.transformPoint("world", depth_point)
  x_adj, y_adj, z_adj = p.point.x, p.point.y, p.point.z
  print(f"x_adj: {x_adj}, y_adj: {y_adj}, z_adj: {z_adj}")
  pub = rospy.Publisher("/marker_test", PointStamped, queue_size=1)
  pub.publish(p)
  

# Initialize a subscriber to the "/camera_rgbd/rgb/image_raw" topic with the image_callback as a callback
depth_points = rospy.Subscriber("/camera_rgbd/depth/points", PointCloud2, callback, queue_size=1)

# Loop to keep the program from shutting down unless ROS is shut down
while not rospy.is_shutdown():
  rospy.spin()
