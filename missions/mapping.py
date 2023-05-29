import cv2
import rospy
import mavros_msgs
from mavros_msgs import srv
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState, PositionTarget
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import BatteryState, NavSatFix, Image
from cv_bridge import CvBridge 
import numpy as np
import math
import time
import sys
from fractions import Fraction
from PIL import Image as img
import piexif
from PIL.ExifTags import TAGS, GPSTAGS
#cap = cv2.VideoCapture(0)

class Mapping():

    def __init__(self):

        self.bridge_object = CvBridge()
        rospy.init_node('mapping_node', anonymous=False)
        self.cam_sub = rospy.Subscriber('/webcam/image_raw', Image, self.cam_callback)
        self.quantidade_fotos = 0
        self.rate = rospy.Rate(0.5)
        self.global_pose = NavSatFix()

        self.global_position_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.global_callback)

    def global_callback(self, global_data):

        self.global_pose = global_data
    

    def cam_callback(self, message):

        self.cam_frame = self.bridge_object.imgmsg_to_cv2(message, "bgr8")                    
            
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
        output = "/home/renato/Documents/tagged/image%d.jpg"%self.quantidade_fotos
        name_clean = "image%d.jpg"%self.quantidade_fotos
        frame = self.cam_frame
        retorno = cv2.imwrite(name, frame)
        self.add_gps_location(name,self.global_pose.latitude, self.global_pose.longitude,output)
        rospy.loginfo("Image " + str(self.quantidade_fotos) + " at lat: " +  str(self.global_pose.latitude) + ", long: " + str(self.global_pose.longitude))
        self.quantidade_fotos += 1
        
    
    def add_gps_location(self,image_path, latitude, longitude, output_path):

        exif_dict = piexif.load(image_path)
        
        # Convert latitude and longitude to degrees, minutes, and seconds
        lat_deg = int(abs(latitude))
        lat_min = int((abs(latitude) - lat_deg) * 60)
        lat_sec = round(((abs(latitude) - lat_deg) * 60 - lat_min) * 60)
        lat_ref = 'N' if latitude >= 0 else 'S'
        
        lon_deg = int(abs(longitude))
        lon_min = int((abs(longitude) - lon_deg) * 60)
        lon_sec = round(((abs(longitude) - lon_deg) * 60 - lon_min) * 60)
        lon_ref = 'E' if longitude >= 0 else 'W'
        
        # Construct the GPS coordinates
        gps_ifd = {
            piexif.GPSIFD.GPSVersionID: (2, 0, 0, 0),
            piexif.GPSIFD.GPSLatitudeRef: lat_ref,
            piexif.GPSIFD.GPSLatitude: ((lat_deg, 1), (lat_min, 1), (lat_sec, 1)),
            piexif.GPSIFD.GPSLongitudeRef: lon_ref,
            piexif.GPSIFD.GPSLongitude: ((lon_deg, 1), (lon_min, 1), (lon_sec, 1)),
        }
        
        # Update the image's EXIF data with the GPS information
        exif_dict['GPS'] = gps_ifd
        
        # Convert the EXIF data back to bytes
        exif_bytes = piexif.dump(exif_dict)
        
        # Save the updated image with the GPS metadata
        piexif.insert(exif_bytes, image_path)
        
        print("GPS metadata added to the image.")

if __name__ == '__main__':

    mapping = Mapping()

    while not rospy.is_shutdown():
        rospy.spin()
       

   