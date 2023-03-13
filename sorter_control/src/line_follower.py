#!/usr/bin/env python
# coding: utf-8

import time
from math import sin, cos
import numpy as np

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from hector_uav_msgs.srv import EnableMotors

import cv2
from cv_bridge import CvBridge, CvBridgeError

from nav_msgs.msg import Odometry
import tf2_ros
import tf.transformations as ttt

PLANING_HORIZON = 50

TIME_LIFTOFF = 3

V_MAX = 2.05
W_MAX = 0.35

Kp_z = 0.5

Kp_y =  0.015
Kd_y =  0.000045
Ki_y =  0.0000825

Kp_w =  0.01555
Kd_w =  0.000095
Ki_w =  0.000165


class SimpleMover():

    def __init__(self):
        rospy.init_node('line_follower', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        self.cv_bridge = CvBridge()

        rospy.on_shutdown(self.shutdown)

        rospy.Subscriber("cam_1/camera/image", Image, self.camera_callback)
        
        rospy.Subscriber('/ground_truth/state', Odometry, self.obom_callback)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.drone_state = [0] * 6  # position vector
        self.e_y = 0
        self.e_omega_z = 0
        self.rate = rospy.Rate(30)

        self.omega_error = 0
        self.y_error = 0


    def obom_callback(self, msg):
        """ Pose of a robot extraction"""
        transform = self.tfBuffer.lookup_transform('world', 'base_stabilized', rospy.Time()).transform
        x, y, z = transform.translation.x, transform.translation.y, transform.translation.z
        quat = transform.rotation
        r, p, y = ttt.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        self.drone_state = [x, y, z, r, p, y]

    def camera_callback(self, msg):
        """ Computer vision stuff"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        grey_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(grey_image, 8, 255, cv2.THRESH_BINARY_INV)
        cv_image = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        cv2.line(cv_image, (160, 0), (160, 240), (0, 123, 0), 1)
        cv2.line(cv_image, (0, 120), (320, 120), (0, 123, 0), 1)

        # "steering" conrol
        top_points = np.where(mask[10] >= 10)
        mid_points = np.where(mask[msg.height / 2] >= 10)
        if  (not np.isnan(np.average(top_points)) and not np.isnan(np.average(mid_points))):
            top_line_point = int(np.average(top_points))
            mid_line_point = int(np.average(mid_points))
            self.omega_error = top_line_point - mid_line_point
            
            cv2.circle(cv_image, (top_line_point, 10), 5, (0,0,255), 1)
            cv2.circle(cv_image, (mid_line_point, int(msg.height/2)), 5, (0,0,255), 1)
            cv2.line(cv_image, (mid_line_point, int(msg.height/2)), (top_line_point, 10), (0, 0, 255), 3)

        # y-offset control
        __, cy_list = np.where(mask >= 10)
        if not np.isnan(np.average(cy_list)):
            cy = int(np.average(cy_list))
            self.y_error = msg.width / 2 - cy
            
            cv2.circle(cv_image, (cy, int(msg.height/2)), 7, (0,255,0), 1)
            cv2.line(cv_image, (160, 120), (cy, int(msg.height/2)), (0, 255, 0), 3)

        self.show_image(cv_image)


    def show_image(self, img, title='Camera 1'):
        cv2.imshow(title, img)
        cv2.waitKey(3)

    def enable_motors(self):
        try:
            rospy.wait_for_service('enable_motors', 2)
            call_service = rospy.ServiceProxy('enable_motors', EnableMotors)
            response = call_service(True)
        except Exception as e:
            print("Error while try to enable motors: ", e)

    def spin(self):
        self.enable_motors()
        
        # Initialisations
        altitude_prev = 0
        y_error_prev = 0
        omega_error_prev = 0

        alpha = self.drone_state[5]

        time_start = rospy.get_time()
        time_prev = time_start
        while not rospy.is_shutdown():
            try:
                # Time stuff
                t = rospy.get_time() - time_start
                dt = t - time_prev
                time_prev = t
                if dt == 0:
                    dt = 1 / 30.

                # TODO: Write here altitude controller
                # Here!
                K = 0.4
                B = 0.1
                z = self.drone_state[2]
                z_des = 3.5
                # z_dot = self.drone_state[5]

                u_z = K * (z_des - z) - B * (z - altitude_prev) / dt
                altitude_prev = self.drone_state[2]

                # TODO: Steering control
                # Here!
                K_s = 0.01
                B_s = 0.00

                u_omega_z = -K_s*self.omega_error - B_s*(self.omega_error - omega_error_prev) / dt
                omega_error_prev = self.omega_error

                # TODO: Offset control
                # Here!
                K_o = 0.01
                B_o = 0.0

                u_y = K_o*self.y_error - B_o*(self.y_error - y_error_prev)/dt
                y_error_prev = self.y_error

                twist_msg = Twist()
                twist_msg.linear.x = 1.5
                twist_msg.linear.y = u_y
                twist_msg.linear.z = u_z
                twist_msg.angular.z = u_omega_z
                self.cmd_vel_pub.publish(twist_msg)

            except KeyboardInterrupt:
                break

            self.rate.sleep()

    def shutdown(self):
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__=="__main__":
    simple_mover = SimpleMover()
    simple_mover.spin()
