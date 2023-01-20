#!/usr/bin/env python3

import cv2
import numpy as np
import socket
import sys
import pickle
import struct ### new code

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

clientsocket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
clientsocket.connect(('localhost',8089))

rospy.init_node('opencv_example', anonymous=True)
bridge = CvBridge()

def stream_image(cv_image):
  frame = cv_image

  data = pickle.dumps(frame) ### new code

  # Send message length first
  message_size = struct.pack("L", len(data)) ### CHANGED

  clientsocket.sendall(message_size + data)

def image_callback(img_msg):
  try:
    cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
  except (CvBridgeError, e):
    rospy.logerr("CvBridge Error: {0}".format(e))

  stream_image(cv_image)
  
sub_image = rospy.Subscriber("/ambf/env/cameras/depth_camera/ImageData", Image, image_callback)
cv2.namedWindow("Image Window", 1)

while not rospy.is_shutdown():
  rospy.spin()