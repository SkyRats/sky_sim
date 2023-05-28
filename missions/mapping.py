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
import exiftool as ex
import pyexiv2
import fractions
#from PIL import Image
from PIL.ExifTags import TAGS
import sys

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
        name_clean = "image%d.jpg"%self.quantidade_fotos
        frame = self.cam_frame
        retorno = cv2.imwrite(name, frame)
        #self.set_gps_location(name,float(self.global_pose.latitude), float(self.global_pose.longitude))
        rospy.loginfo("Image " + str(self.quantidade_fotos) + " at lat: " +  str(self.global_pose.latitude) + ", long: " + str(self.global_pose.longitude))
        self.quantidade_fotos += 1
      
        
    def to_deg(self, value, loc):
        if value < 0: 
            loc_value = loc[0]
        elif value > 0:
            loc_value = loc[1]
        else:
            loc_value = ""
        abs_value = abs(value)
        deg =  int(abs_value)
        t1 = (abs_value-deg)*60
        min = int(t1)
        sec = round((t1 - min)* 60, 5)
        return (deg, min, sec, loc_value)    

    def set_gps_location(self,file_name, lat, lng):
        """Adds GPS position as EXIF metadata

        Keyword arguments:
        file_name -- image file
        lat -- latitude (as float)
        lng -- longitude (as float)

        """
        lat_deg = self.to_deg(lat, ["S", "N"])
        lng_deg = self.to_deg(lng, ["W", "E"])


        # convert decimal coordinates into degrees, munutes and seconds
        exiv_lat = (pyexiv2.Rational(lat_deg[0]*60+lat_deg[1],60),pyexiv2.Rational(lat_deg[2]*100,6000), pyexiv2.Rational(0, 1))
        exiv_lng = (pyexiv2.Rational(lng_deg[0]*60+lng_deg[1],60),fractions.Fraction(lng_deg[2]*100,6000), pyexiv2.Rational(0, 1))
        metadata = pyexiv2.ImageMetadata(file_name)
        metadata.read()
    
    ##    exif_keys = metadata.exif_keys

        metadata["Exif.GPSInfo.GPSLatitude"] = exiv_lat
        metadata["Exif.GPSInfo.GPSLatitudeRef"] = lat_deg[3]
        metadata["Exif.GPSInfo.GPSLongitude"] = exiv_lng
        metadata["Exif.GPSInfo.GPSLongitudeRef"] = lng_deg[3]
        metadata["Exif.Image.GPSTag"] = 654
        metadata["Exif.GPSInfo.GPSMapDatum"] = "WGS-84"
        metadata["Exif.GPSInfo.GPSVersionID"] = '2 0 0 0'

        metadata.write()

if __name__ == '__main__':

    mapping = Mapping()

    while not rospy.is_shutdown():
        rospy.spin()
       

   