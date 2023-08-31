#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import time
from geometry_msgs.msg import TwistStamped, Vector3, Point, PoseStamped
from mavros_msgs.msg import PositionTarget

from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tf2_msgs.msg import TFMessage

import math


class image_converter:

    def __init__(self):
        self.pub_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
        self.pub_ang_vel = rospy.Publisher('/mavros/setpoint_attitude/cmd_vel', TwistStamped, queue_size=1)
        self.pub_error = rospy.Publisher('error', Int16, queue_size=10)
        self.pub_angle = rospy.Publisher('angle', Int16, queue_size=10)
        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber('/webcam/image_raw', Image, self.callback)

        self.setpoint_pub_ = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        rospy.Subscriber('/tf', TFMessage, self.callback)


        self.Kp = 0.112                 # Ku=0.14 T=6. PID: p=0.084,i=0.028,d=0.063. PD: p=0.112, d=0.084/1. P: p=0.07
        self.Ki = 0
        self.kd = 1
        self.integral = 0
        self.derivative = 0
        self.last_error = 0
        self.Kp_ang = 0.01             # Ku=0.04 T=2. PID: p=0.024,i=0.024,d=0.006. PD: p=0.032, d=0.008. P: p=0.02/0.01
        self.Ki_ang = 0
        self.kd_ang = 0
        self.integral_ang = 0
        self.derivative_ang = 0
        self.last_ang = 0
        self.was_line = 0
        self.line_side = 0
        self.line_back = 1
        self.error = []
        self.angle = []
        self.fly_time = 0.0
        self.start = 0.0
        self.stop = 0.0
        self.velocity = 0.1
        self.capture = cv2.VideoCapture(0)

        self.drone_pos_ = Point()


    def callback(self, data):
        for transform in data.transforms:
            if transform.child_frame_id == "camera_link":
                self.drone_pos_.x = transform.transform.translation.x
                self.drone_pos_.y = transform.transform.translation.y
                self.drone_pos_.z = transform.transform.translation.z
                break

    def line_detect(self, cv_image):

        lower_mask = np.array([ 69, 69, 37])
        upper_mask = np.array(  [ 157, 255, 255])
        mask = cv2.inRange(cv_image, lower_mask, upper_mask)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=5)
        mask = cv2.dilate(mask, kernel, iterations=9)

        # cv2.imshow("mask", mask)
        # cv2.waitKey(1) & 0xFF

        contours_blk, _ = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_blk = list(contours_blk)

        if len(contours_blk) > 0:
            contours_blk.sort(key=cv2.minAreaRect)
            if cv2.contourArea(contours_blk[0]) > 5000:

                self.was_line = 1
                blackbox = cv2.minAreaRect(contours_blk[0])
                (x_min, y_min), (w_min, h_min), angle = blackbox
                if angle < -45:
                    angle = 90 + angle
                if w_min < h_min and angle > 0:
                    angle = (90 - angle) * -1
                if w_min > h_min and angle < 0:
                    angle = 90 + angle

                angle += 90

                if angle > 90:
                    angle = angle - 180

                setpoint = cv_image.shape[1] / 2
                error = int(x_min - setpoint)
                self.error.append(error)
                self.angle.append(angle)
                normal_error = float(error) / setpoint

                if error > 0:
                    self.line_side = 1  # line in right
                elif error <= 0:
                    self.line_side = -1  # line in left

                self.integral = float(self.integral + normal_error)
                self.derivative = normal_error - self.last_error
                self.last_error = normal_error


                error_corr = -1 * (self.Kp * normal_error + self.Ki * self.integral + self.kd * self.derivative)  # PID controler
                # print("error_corr:  ", error_corr, "\nP", normal_error * self.Kp, "\nI", self.integral* self.Ki, "\nD", self.kd * self.derivative)

                angle = int(angle)

                self.integral_ang = float(self.integral_ang + angle)
                self.derivative_ang = angle - self.last_ang
                self.last_ang = angle

                ang_corr = -1 * (self.Kp_ang * angle + self.Ki_ang * self.integral_ang + self.kd_ang * self.derivative_ang)  # PID controler

                box = cv2.boxPoints(blackbox)
                box = np.intp(box)

                # cv2.drawContours(cv_image, [box], 0, (0, 0, 255), 3)

                # cv2.putText(cv_image, "Angle: " + str(angle), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2,
                #             cv2.LINE_AA)

                # cv2.putText(cv_image, "Error: " + str(error), (10, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2,
                #             cv2.LINE_AA)
                # cv2.line(cv_image, (int(x_min), 200), (int(x_min), 250), (255, 0, 0), 3)


                setpoint_ = PositionTarget()
                vel_setpoint_ = Vector3()

                vel_setpoint_.x = self.velocity
                vel_setpoint_.y = error_corr

                yaw_setpoint_ = ang_corr * math.pi / 180 * 5

                print(f"x: {vel_setpoint_.x} | y: {vel_setpoint_.y} | yaw: {yaw_setpoint_}")

                setpoint_.coordinate_frame = PositionTarget.FRAME_BODY_NED
                
                setpoint_.velocity.x = vel_setpoint_.x
                setpoint_.velocity.y = vel_setpoint_.y
                # setpoint_.velocity.x = 0
                # setpoint_.velocity.y = 0
                setpoint_.position.z = self.drone_pos_.z
                



                setpoint_.yaw = yaw_setpoint_

                self.setpoint_pub_.publish(setpoint_)

                ang = Int16()
                ang.data = angle
                self.pub_angle.publish(ang)

                err = Int16()
                err.data = error
                self.pub_error.publish(err)


    # Zoom-in the image
    def zoom(self, cv_image, scale):
        if cv_image is None: return None
        height, width, _ = cv_image.shape
        # print(width, 'x', height)
        # prepare the crop
        centerX, centerY = int(height / 2), int(width / 2)
        radiusX, radiusY = int(scale * height / 100), int(scale * width / 100)

        minX, maxX = centerX - radiusX, centerX + radiusX
        minY, maxY = centerY - radiusY, centerY + radiusY

        cv_image = cv_image[minX:maxX, minY:maxY]
        cv_image = cv2.resize(cv_image, (width, height))

        return cv_image


    # Image processing @ 10 FPS
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image = self.zoom(cv_image, scale=20)
        cv_image = cv2.add(cv_image, np.array([-50.0]))
        # height, width, _ = cv_image.shape
        # print(width, 'x', height)
        # if self.takeoffed and (not self.landed):
        cv_image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        self.line_detect(cv_image_hsv)
            # self.land_detect(cv_image)


        # cv2.putText(cv_image, "battery: " + str(self.battery) + "%", (570, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
        #             (255, 255, 0), 2, cv2.LINE_AA)

        # cv2.imshow("Image window", cv_image)
        # cv2.imshow("mask", mask)
        # cv2.waitKey(1) & 0xFF

    def detection_loop(self):
        while self.capture.isOpened():
            
            ret, cv_image = self.capture.read()
            if ret:
                cv_image = self.zoom(cv_image, scale=20)
                cv_image = cv2.add(cv_image, np.array([-50.0]))
                # height, width, _ = cv_image.shape
                # print(width, 'x', height)
                # if self.takeoffed and (not self.landed):
                cv_image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
                self.line_detect(cv_image_hsv)

            else:
                break
        


def main():
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    time.sleep(3)
    try:
        ic.detection_loop()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
