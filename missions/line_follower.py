#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import time
from dronekit import connect, VehicleMode

from std_msgs.msg import Int16  # For error/angle plot publishing
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pymavlink import mavutil


class image_converter:

    def __init__(self):
        self.pub_error = rospy.Publisher('error', Int16, queue_size=10)
        self.pub_angle = rospy.Publisher('angle', Int16, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/webcam/image_raw', Image, self.callback)

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


    def set_velocity(self, vx, vy, vz, w):
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target_system, target_component
            mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions
            vx, vy, vz, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, w)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        # send command to vehicle
        vehicle.send_mavlink(msg)

    def set_yaw_velocity(self, w, relative=False):
        if relative:
            is_relative=1 #yaw relative to direction of travel
        else:
            is_relative=0 #yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            w,    # param 1, yaw in degrees
            10,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        vehicle.send_mavlink(msg)

    # Detect the line and piloting
    def line_detect(self, cv_image):
        # Create a mask
        # cv_image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)Lower mask:
 
        lower_mask = np.array((86, 85, 56))
        upper_mask = np.array( (128, 255, 255))
        mask = cv2.inRange(cv_image, lower_mask, upper_mask)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=5)
        mask = cv2.dilate(mask, kernel, iterations=9)

        cv2.imshow("mask", mask)
        cv2.waitKey(1) & 0xFF

        contours_blk, _ = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Sort by X value (takes the contours from left to right)
        # print(contours_blk)
        # print(babs)

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
                print(angle)
                normal_ang = float(angle) / 90

                self.integral_ang = float(self.integral_ang + angle)
                self.derivative_ang = angle - self.last_ang
                self.last_ang = angle

                ang_corr = -1 * (self.Kp_ang * angle + self.Ki_ang * self.integral_ang + self.kd_ang * self.derivative_ang)  # PID controler

                box = cv2.boxPoints(blackbox)
                box = np.intp(box)

                cv2.drawContours(cv_image, [box], 0, (0, 0, 255), 3)

                cv2.putText(cv_image, "Angle: " + str(angle), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2,
                            cv2.LINE_AA)

                cv2.putText(cv_image, "Error: " + str(error), (10, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2,
                            cv2.LINE_AA)
                cv2.line(cv_image, (int(x_min), 200), (int(x_min), 250), (255, 0, 0), 3)

                self.set_velocity(0, 0, 0, ang_corr)
                self.set_yaw_velocity(ang_corr)

                # print("angVal: ", twist.angular.z)

                ang = Int16()
                ang.data = angle
                self.pub_angle.publish(ang)

                err = Int16()
                err.data = error
                self.pub_error.publish(err)

        # if len(contours_blk) == 0 and self.was_line == 1 and self.line_back == 1:
        #     twist = Twist()
        #     twistStamped = TwistStamped()
                
        #     if self.line_side == 1:  # line at the right
        #         twist.linear.y = -0.05
        #         twistStamped.twist = twist
        #         self.pub_vel.publish(twistStamped)
        #     if self.line_side == -1:  # line at the left
        #         twist.linear.y = 0.05
        #         twistStamped.twist = twist
        #         self.pub_vel.publish(twistStamped)
        # cv2.imshow("mask", mask)
        # cv2.waitKey(1) & 0xFF

    # def land_detect(self, cv_image):
    #     land_mask = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    #     land_mask = cv2.GaussianBlur(land_mask, (21, 21), 0)
    #     low_red = np.array([0, 64, 148])
    #     up_red = np.array([19, 221, 229])
    #     land_mask = cv2.inRange(land_mask, low_red, up_red)

    #     _, contours_blk2, _ = cv2.findContours(land_mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #     contours_blk2.sort(key=cv2.minAreaRect)
    #     # cv2.imshow("land mask", land_mask)
    #     if len(contours_blk2) > 0 and cv2.contourArea(contours_blk2[0]) > 30000:
    #         self.landing()

    # Landing the drone
    # def landing(self):
    #     land = Empty()
    #     self.pub_land.publish(land)
    #     # self.takeoffed = 0
    #     # self.stop = time.time()
    #     # self.errorPlot()

    # def isTakeoff(self, data):
    #     time.sleep(3.5)
    #     self.start = time.time()
    #     self.takeoffed = 1
    #     self.landed = 0

    # def isLand(self, data):
    #     self.landed = 1
    #     self.takeoffed = 0
    #     self.stop = time.time()
    #     self.errorPlot()

    # def errorPlot(self):
    #     meanError = np.mean(self.error)
    #     stdError = np.std(self.error)
    #     meanAngle = np.mean(self.angle)
    #     stdAngle = np.std(self.angle)
    #     self.fly_time = self.stop - self.start
    #     print("""
    #   /*---------------------------------------------
    #           meanError: %f[px],   stdError: %f[px]
    #           meanAngle: %f[deg],   stdAngle: %f[deg]
    #           Time: %f[sec], Velocity: %f[percent]
    #   ---------------------------------------------*/
    #   """ %(meanError, stdError, meanAngle, stdAngle, self.fly_time, self.velocity))

    # Zoom-in the image
    def zoom(self, cv_image, scale):
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

        cv2.imshow("Image window", cv_image)
        # cv2.imshow("mask", mask)
        cv2.waitKey(1) & 0xFF


def main():
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    time.sleep(3)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    connection_string = '127.0.0.1:14550'
    vehicle = connect(connection_string)
    main()