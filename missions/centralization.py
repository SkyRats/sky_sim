import rospy
from sensor_msgs.msg import Image
import cv2
import time
import numpy as np
from cv2 import aruco
from dronekit import connect, VehicleMode
from pymavlink import mavutil
from cv_bridge import CvBridge 
import argparse
from simple_pid import PID

class MarkerDetector:
    def __init__(self, target_type, target_size, camera_info):

        self.target_type = target_type
        self.marker_size = target_size
        self.mode = "Searching"
        if self.target_type == 'aruco':
            self.dictionary = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
            self.parameters =  aruco.DetectorParameters()
            self.detector = aruco.ArucoDetector(self.dictionary, self.parameters)

        elif self.target_type == 'qrcode':
            print("QR Code not implemented yet!")

        self.camera_matrix = camera_info[0]
        self.dist_coeff = camera_info[1]
        
        self.np_camera_matrix = np.array(self.camera_matrix)
        self.np_dist_coeff = np.array(self.dist_coeff)

        self.horizontal_res = camera_info[2][0]
        self.vertical_res = camera_info[2][1]

        self.horizontal_fov = camera_info[3][0]
        self.vertical_fov = camera_info[3][1]


    def pose_estimation(self, corners, marker_size, mtx, distortion):
        '''
        This will estimate the rvec and tvec for each of the marker corners detected by:
        corners, ids, rejectedImgPoints = detector.detectMarkers(image)
        corners - is an array of detected corners for each detected marker in the image
        marker_size - is the size of the detected markers
        mtx - is the camera matrix
        distortion - is the camera distortion matrix
        RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
        '''
        marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, -marker_size / 2, 0],
                                [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    
        nada, rvec, tvec = cv2.solvePnP(marker_points, corners, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        return rvec, tvec
    
    def aruco_detection(self, frame):

        # Marker detection
        markerCorners, markerIds, rejected = self.detector.detectMarkers(frame)

        i = 0
        if len(markerCorners) > 0: # if detect any Arucos

            closest_target = []
            closest_dist = 100000 # 1000 m (arbitrary large value)

            for corners in markerCorners: # For each Aruco

                marker_points = corners[0] # Vector with 4 points (x, y) for the corners
                # Draw points in image
                final_image = self.draw_marker(frame, marker_points)
                #cv2.imshow('detected', final_image)
                i += 1
                
            
            return markerCorners, final_image
        return None
    
    def draw_marker(self, frame, points):
        topLeft, topRight, bottomRight, bottomLeft = points

        # Marker corners
        tR = (int(topRight[0]), int(topRight[1]))
        bR = (int(bottomRight[0]), int(bottomRight[1]))
        bL = (int(bottomLeft[0]), int(bottomLeft[1]))
        tL = (int(topLeft[0]), int(topLeft[1]))
        w = np.sqrt((topRight[0]-topLeft[0])**2 + (topRight[1]-topLeft[1])**2)
        h = np.sqrt((topRight[0]-bottomRight[0])**2 + (topRight[1]-bottomRight[1])**2)
        # Find the Marker center
        cX = int((tR[0] + bL[0]) / 2.0)
        cY = int((tR[1] + bL[1]) / 2.0)

        # Draw rectangle and circle
        rect = cv2.rectangle(frame, tL, bR, (0, 0, 255), 2)
        circle = cv2.circle(rect, (cX, cY), radius=4, color=(0, 0, 255), thickness=-1)
        final = cv2.putText(circle,"mode: " + self.mode,(20,20), cv2.FONT_HERSHEY_SIMPLEX,1, (255, 255, 255), 2, cv2.LINE_AA)        
        print(topLeft[1])
        self.goal = [topLeft[0]+ w/2,topLeft[1]+h/2]

        return final


class Centralize:
    def __init__(self, vehicle, target_type, target_size, camera_info):

        # Drone
        self.vehicle = vehicle
        
        # Marker detector object
        self.detector = MarkerDetector(target_type, target_size, camera_info)

        # ROS node
        rospy.init_node('drone_node', anonymous=False)
        self.detector.mode = "Searching"
        self.cap = cv2.VideoCapture(0)


    def move_drone_with_velocity(self, vx, vy, vz, duration):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,  # time_boot_ms, target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
            0b0000111111000111,  # type_mask (ignore everything except position and yaw)
            0, 0, 0,  # x, y, z positions (not used)
            vx, vy, vz,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not used)
            0, 0)  # yaw, yaw_rate (not used)

        self.vehicle.send_mavlink(msg)
        
    
    def visual_servoing_control(self,corners, frame):     

        # Initialize PID controllers for lateral and forward control
        pid_x = PID(Kp=0.007, Ki=0.005, Kd=0.005, setpoint=0)
        pid_y = PID(Kp=0.007, Ki=0.005, Kd=0.005, setpoint=0)

        if corners:
            # Assuming the first detected marker's center is our target
            marker_center = corners[0][0].mean(axis=0)

            # Calculate the error between the marker center and the image center
            image_center = np.array([frame.shape[1] / 2, frame.shape[0] / 2])
            error = marker_center - image_center
            print(error)
            # Calculate the desired velocity commands (lateral and forward) using PID controllers
            vx = +pid_x(error[1])  # Drone moves in the opposite direction to align with the marker's center (left/right)
            vy = -pid_y(error[0])

            # Visual servoing parameters (adjust as needed)
            vz = 0  # Desired vertical velocity (m/s)
            duration = 0.5 # Duration of each movement command (in seconds)

            # Move the drone with velocity commands
            self.move_drone_with_velocity(vx, vy, vz, duration)


    #-- Callback
    def detection_loop(self):

        while self.cap.isOpened():
            #frame = self.bridge_object.imgmsg_to_cv2(message,"bgr8")
            success,frame = self.cap.read()
            #frame = cv2.flip(frame,1)
            #cv2.imshow("frame", frame)
            # Look for the closest target in the frame
                      
            aruco = self.detector.aruco_detection(frame)
            if aruco is not None:
                print("cheguei")
                self.detector.mode = "Tracking"
                corners, draw_img = aruco
                self.visual_servoing_control(corners,frame)

            k = cv2.waitKey(30) & 0xff
            if k == 27:
                break

        self.cap.release()
        cv2.destroyAllWindows()
if __name__ == '__main__':

    #-- SETUP

    # Target size in cm
    marker_size = 20

    # Camera infos
    camera_matrix = [[467.74270306499267, 0.0, 320.5],
                    [0.0, 467.74270306499267, 240.5],
                    [0.0, 0.0, 1.0]]

    dist_coeff = [0.0, 0.0, 0.0, 0.0, 0] # Camera distortion matrix
    res = (640, 512) # Camera resolution in pixels
    fov = (1.2, 1.1) # Camera FOV

    camera = [camera_matrix, dist_coeff, res, fov]


    #-- DRONEKIT

    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', default = '127.0.0.1:14550')
    args = parser.parse_args()

    #-- Connect to the vehicle
    print('Connecting...')
    vehicle = connect(args.connect)

    #-- Check vehicle status
    print(f"Mode: {vehicle.mode.name}")
    print(" Global Location: %s" % vehicle.location.global_frame)
    print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    print(" Local Location: %s" % vehicle.location.local_frame)
    print(" Attitude: %s" % vehicle.attitude)
    print(" Velocity: %s" % vehicle.velocity)
    print(" Gimbal status: %s" % vehicle.gimbal)
    print(" EKF OK?: %s" % vehicle.ekf_ok)
    print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
    print(" Rangefinder: %s" % vehicle.rangefinder)
    print(" Rangefinder distance: %s" % vehicle.rangefinder.distance)
    print(" Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
    print(" Is Armable?: %s" % vehicle.is_armable)
    print(" System status: %s" % vehicle.system_status.state)
    print(" Armed: %s" % vehicle.armed)    # settable

    #-- DRONEKIT 1 bugado, arrumar parâmetros manualmente!
    # vehicle.parameters['PLND_ENABLED']      = 1
    # vehicle.parameters['PLND_TYPE']         = 1 # Mavlink landing backend
    # vehicle.parameters['LAND_REPOSITION']   = 0 # !!!!!! ONLY FOR SITL IF NO RC IS CONNECTED
    # print("Parâmtros ok!")
        #ros_img = self.bridge_object.cv2_to_imgmsg(draw_img, 'bgr8')

    # arm_and_takeoff(10)
    # print("Take off complete")
    # time.sleep(10)

    if vehicle.mode != 'GUIDED':
        vehicle.mode = VehicleMode('GUIDED')
        while vehicle.mode != 'GUIDED':
            time.sleep(1)
        print('vehicle in LAND mode')

    print("Going for precision landing...")
    centralize = Centralize(vehicle, 'aruco', marker_size, camera)
    centralize.detection_loop()
    print("END")
    vehicle.close()
