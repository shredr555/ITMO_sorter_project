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
    self.image_sub = rospy.Subscriber('/sorter/camera1/image_raw', Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('/sorter/cmd_vel', Twist, queue_size=1)
    self.twist = Twist()
        

  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.asarray([0, 0, 255])
    upper_yellow = numpy.asarray([30, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 150
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    M = cv2.moments(mask)
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      
      err = cx - w/4
      self.twist.linear.x = -0.8
      self.twist.angular.z = float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
    # else:
    #   self.twist.linear.x = 0.3
    #   self.cmd_vel_pub.publish(self.twist)
    
    # cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
    cv2.imshow("mask",mask)
    cv2.imshow("output", image)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()