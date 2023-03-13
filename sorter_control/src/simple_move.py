#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist


class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    #cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('/sorter/camera1/image_raw', 
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
                                       Twist, queue_size=1)
    self.twist = Twist()
        

  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
    # change below lines to map the color you wanted robot to follow
    lower_yellow = numpy.array([ 44,  100,  100])
    upper_yellow = numpy.array([80, 100, 100])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    M = cv2.moments(mask)
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      #cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      # CONTROL starts
      err = cx - w/2
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
      # CONTROL ends
    else:
      self.twist.linear.x = 1
      self.cmd_vel_pub.publish(self.twist)
      
    cv2.imshow("mask",mask)
    cv2.imshow("output", image)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()