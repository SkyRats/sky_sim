import rospy
from sensor_msgs.msg import Image
import cv2
import time
import numpy as np
import ros_numpy as rnp
from cv2 import aruco
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
from cv_bridge import CvBridge 
import argparse  


#-- SETUP

bridge_object = CvBridge()

# Tamanho da base em cm
marker_size = 100

# Distorção da câmera
dist_coeff = [0.0, 0.0, 0.0, 0.0, 0] 

# Câmera matrix -> tópico de informações da câmera
camera_matrix = [[467.74270306499267, 0.0, 320.5], [0.0, 467.74270306499267, 240.5], [0.0, 0.0, 1.0]]
np_camera_matrix = np.array(camera_matrix)
np_dist_coeff = np.array(dist_coeff)

horizontal_res = 640
vertical_res = 480

horizontal_fov = 1.2
vertical_fov = 1.1

arucoParams = cv2.aruco.DetectorParameters_create()
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)

# Publisher da imagem após a detecção
newimg_pub = rospy.Publisher('camera/colour/image_new', Image, queue_size=10)
cam = Image()


#-- Callback

def msg_receiver(message):

    # Bridge de ROS para CV
    cam = bridge_object.imgmsg_to_cv2(message,"bgr8")

    frame = cam

    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)

    # (corners, ids, rejected) = aruco.detectMarkers(gray_img, aruco_dict, parameters)
    ret = aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix=np_camera_matrix, distCoeffs=np_dist_coeff)

    if len(corners) > 0:
            
        ids = ids.flatten()

        for (markerCorner, markerID) in zip(corners, ids):

            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            # Marker corners
            tR = (int(topRight[0]), int(topRight[1]))
            bR = (int(bottomRight[0]), int(bottomRight[1]))
            bL = (int(bottomLeft[0]), int(bottomLeft[1]))
            tL = (int(topLeft[0]), int(topLeft[1]))

            # Find the Marker center
            cX = int((tR[0] + bL[0]) / 2.0)
            cY = int((tR[1] + bL[1]) / 2.0)

            rect = cv2.rectangle(frame, tL, bR, (0, 0, 255), 2)
            final = cv2.circle(rect, (cX, cY), radius=4, color=(0, 0, 255), thickness=-1)

            (rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])
            x="{:.2f}".format(tvec[0])
            y="{:.2f}".format(tvec[1])
            z="{:.2f}".format(tvec[2])

            x_sum = 0
            y_sum = 0

            x_sum = corners[0][0] + corners[1][0] + corners[2][0] + corners[3][0]
            y_sum = corners[0][1] + corners[1][1] + corners[2][1] + corners[3][1]

            x_avg = x_sum / 4
            y_avg = y_sum / 4

            x_ang = (x_avg - horizontal_res*0.5)*horizontal_fov/horizontal_res
            y_ang = (y_avg - vertical_res*0.5)*vertical_fov/vertical_res

            if vehicle.mode != 'LAND':
                vehicle.mode = VehicleMode('LAND')
                while vehicle.mode != 'LAND':
                    time.sleep(1)
                print('vehicle in LAND mode')
            
        # times = time.time()*1e6
        dist = float(z)/100
        send_land_message(x_ang, y_ang, dist)

        # print(f'MARKER POSITION: x={x} | y={y} | z={z} | x_ang={x_ang} | y_ang={y_ang}')
        print(f'MARKER POSITION: x_ang={round(x_ang, 2)} | y_ang={round(y_ang, 2)} | z={round(dist,2)}')

        ros_img = bridge_object.cv2_to_imgmsg(final, 'bgr8')
        newimg_pub.publish(ros_img)


##############################################################

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


#-- Landing Target Protocol

def send_land_message(x_ang,y_ang,dist_m,time=0):
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
    print('mensagem enviada!')


def subscriber():
    rospy.init_node('drone_node', anonymous=False)
    sub = rospy.Subscriber('/webcam/image_raw', Image, msg_receiver)
    rospy.spin()

try:
    print("Criando subscriber...")
    subscriber()
except:
    print('Erro!')
