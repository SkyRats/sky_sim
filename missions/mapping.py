import rospy
import cv2
import time
import numpy as np
from cv2 import aruco
#from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
from cv_bridge import CvBridge, CvBridgeError
import argparse  
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
import math



#cap = cv2.VideoCapture(0)

class Mapping():

    def __init__(self):

        self.bridge_object = CvBridge()
        rospy.init_node('drone_node', anonymous=False)
        self.cam_sub = rospy.Subscriber('/webcam/image_raw', Image, self.cam_callback)
        self.quantidade_fotos = 0
        self.rate = rospy.Rate(0.5)
        
    def cam_callback(self, message):

        try:
            self.cam_frame = self.bridge_object.imgmsg_to_cv2(message, "bgr8")                    
            
        except CvBridgeError as e:
            print(e)

        print("cheguei")
        frame = self.cam_frame
        #print(frame)
        #cv2.imshow("frame")
        rospy.loginfo("Message received")
        self.save_pictures()
        self.rate.sleep()

    def save_pictures(self):

        #print("cheguei")
        name = "/home/renato/Documents/Images/image%d.jpg"%self.quantidade_fotos
        name_clean = "image%d.jpg"%self.quantidade_fotos
        frame = self.cam_frame
        retorno = cv2.imwrite(name, frame)
        #self.store_gps_coordinate(name_clean)
        rospy.loginfo("Foto %d"%self.quantidade_fotos)
        self.quantidade_fotos += 1
        
            

if __name__ == '__main__':

    mapping = Mapping()

    while not rospy.is_shutdown():
        rospy.spin()
       

   