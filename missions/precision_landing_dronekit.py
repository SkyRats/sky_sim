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


class MarkerDetector:
    def __init__(self, target_type, target_size, camera_info):

        self.target_type = target_type
        self.marker_size = target_size

        if self.target_type == 'aruco':
            self.dictionary = aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
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

                # Pose estimation
                pose = self.pose_estimation(marker_points, self.marker_size, self.np_camera_matrix, self.np_dist_coeff)

                rvec, tvec = pose

                # 3D pose estimation vector
                x = round(tvec[0][0], 2)
                y = round(tvec[1][0], 2)
                z = round(tvec[2][0], 2)

                x_sum = marker_points[0][0] + marker_points[1][0] + marker_points[2][0] + marker_points[3][0]
                y_sum = marker_points[0][1] + marker_points[1][1] + marker_points[2][1] + marker_points[3][1]

                x_avg = x_sum / 4
                y_avg = y_sum / 4

                x_ang = (x_avg - self.horizontal_res*0.5)*self.horizontal_fov/self.horizontal_res
                y_ang = (y_avg - self.vertical_res*0.5)*self.vertical_fov/self.vertical_res

                payload = markerIds[i][0]
                i += 1
                
                # Check for the closest target
                if z < closest_dist:
                    closest_dist = z
                    closest_target = [x, y, z, x_ang, y_ang, payload, final_image]
            
            return closest_target
        return None
    
    def draw_marker(self, frame, points):
        topLeft, topRight, bottomRight, bottomLeft = points

        # Marker corners
        tR = (int(topRight[0]), int(topRight[1]))
        bR = (int(bottomRight[0]), int(bottomRight[1]))
        bL = (int(bottomLeft[0]), int(bottomLeft[1]))
        tL = (int(topLeft[0]), int(topLeft[1]))

        # Find the Marker center
        cX = int((tR[0] + bL[0]) / 2.0)
        cY = int((tR[1] + bL[1]) / 2.0)

        # Draw rectangle and circle
        rect = cv2.rectangle(frame, tL, bR, (0, 0, 255), 2)
        final = cv2.circle(rect, (cX, cY), radius=4, color=(0, 0, 255), thickness=-1)

        return final


class PrecLand:
    def __init__(self, vehicle, target_type, target_size, camera_info):

        # Drone
        self.vehicle = vehicle
        
        # Marker detector object
        self.detector = MarkerDetector(target_type, target_size, camera_info)

        # ROS node
        rospy.init_node('drone_node', anonymous=False)

        # Bridge ros-opencv
        self.bridge_object = CvBridge()

        # Post detection image publisher
        self.newimg_pub = rospy.Publisher('camera/colour/image_new', Image, queue_size=10)
        self.cam = Image()

        try:
            print("Criando subscriber...")
            self.subscriber = rospy.Subscriber('/webcam/image_raw', Image, self.msg_receiver)
        except:
            print('Erro ao criar subscriber!')

        self.teste = []

    def send_land_message(self, x_ang,y_ang,dist_m,time=0):
        msg = vehicle.message_factory.landing_target_encode(
            time,
            0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            x_ang,
            y_ang,
            dist_m,
            0,
            0,
        )
        vehicle.send_mavlink(msg)
        print("Mensagem enviada")

    #-- Callback
    def msg_receiver(self, message):

        # Bridge de ROS para CV
        cam = self.bridge_object.imgmsg_to_cv2(message,"bgr8")
        frame = cam

        # Look for the closest target in the frame
        closest_target = self.detector.aruco_detection(frame)

        if closest_target is not None and self.vehicle.mode == 'LAND':
            
            x, y, z, x_ang, y_ang, payload, draw_img = closest_target

            # times = time.time()*1e6
            dist = float(z)/100
            self.send_land_message(x_ang, y_ang, dist)

            # Simulate tracking delay
            
            # if str(time.time())[11] == '0':
            #     # times = time.time()*1e6
            #     dist = float(z)/100
            #     self.teste = closest_target
            #     self.send_land_message(x_ang, y_ang, dist)
            #     print(x)
            # else:
            #     if len(self.teste) > 0:
            #         self.send_land_message(self.teste[3], self.teste[4], float(self.teste[2])/100)
            #         print(self.teste[0])

            print(f'MARKER POSITION: x = {x} | y = {y} | z = {z} | x_ang = {round(x_ang, 2)} | y_ang = {round(y_ang, 2)} | ID = {payload}')
            
            # Publish image with target identified
            ros_img = self.bridge_object.cv2_to_imgmsg(draw_img, 'bgr8')
            self.newimg_pub.publish(ros_img)


# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
            
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Check that vehicle has reached takeoff altitude
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.        
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude*0.95: 
            print(f"Reached target altitude {vehicle.location.global_relative_frame.alt}")
        break


if __name__ == '__main__':

    #-- SETUP

    # Target size in cm
    marker_size = 50

    # Camera infos
    camera_matrix = [[467.74270306499267, 0.0, 320.5],
                    [0.0, 467.74270306499267, 240.5],
                    [0.0, 0.0, 1.0]]

    dist_coeff = [0.0, 0.0, 0.0, 0.0, 0] # Camera distortion matrix
    res = (640, 480) # Camera resolution in pixels
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

    # arm_and_takeoff(10)
    # print("Take off complete")
    # time.sleep(10)

    if vehicle.mode != 'LAND':
        vehicle.mode = VehicleMode('LAND')
        while vehicle.mode != 'LAND':
            time.sleep(1)
        print('vehicle in LAND mode')

    print("Going for precision landing...")
    precision_landing = PrecLand(vehicle, 'aruco', marker_size, camera)
    rospy.spin()

    print("END")
    vehicle.close()
