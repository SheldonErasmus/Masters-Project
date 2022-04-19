#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo, NavSatFix, Imu
from wgs2ned import WGS2NED
from ToEulerAngles import ToEulerAng
import cv2
from cv_bridge import CvBridge, CvBridgeError
import pyrealsense2 as rs2
from hexapodC import HexapodC
from my_message.msg import PathVar_n_cmdVel, WorldFeetPlace
from std_msgs.msg import Float32

class ImageListener:
    def __init__(self, topic):
        self.topic = topic
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic, msg_Image, self.imageDepthCallback)
        self.sub_info = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.imageDepthInfoCallback)
        self.intrinsics = None
        self.cv_image = [np.zeros((720,1280)) for i in range(4)]
        self.flag = 0

        pose = {"x":0,"y":0, "z":0,"yaw":0}
        self.image_pose = [pose for i in range(4)]

        self.SomeTable = {}

    def imageDepthCallback(self, data):
        if self.flag == 1:
            self.flag = 0 
            try:
                temp = [self.bridge.imgmsg_to_cv2(data, data.encoding)]
                self.cv_image = np.concatenate((self.cv_image[1:4],temp))
                pose = [{"x":n,"y":-e, "z":-d,"yaw":yaw}]
                self.image_pose = np.concatenate((self.image_pose[1:4],pose))
                self.SomeTable = {}
            except CvBridgeError as e:
                print(e)
                return

    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [0, 0 ,0 ,0 ,0] #[i for i in cameraInfo.D]
        except CvBridgeError as e:
            print(e)
            return

startupFlag = 1
n = 0.0
e = 0.0
d = 0.0
def nav_cb(msg):
    global startupFlag, refLat, refLon, refAlt, n, e, d
    if startupFlag:
        startupFlag = 0
        refLat = msg.latitude
        refLon = msg.longitude
        refAlt = msg.altitude
    Lat = msg.latitude
    Lon = msg.longitude
    Alt = msg.altitude
    (n,e,d) = WGS2NED(refLat,refLon,refAlt,Lat,Lon,Alt)

roll=0;pitch=0;yaw=0
def imu_cb(msg):
    global roll,pitch,yaw
    (roll,pitch,yaw) = ToEulerAng(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)

FeetPlaceXBuffer = np.empty((2,6))
FeetPlaceYBuffer = np.empty((2,6))
def FeetPlace_cb(msg):
    global FeetPlaceXBuffer, FeetPlaceYBuffer
    FeetPlaceXBuffer = msg.XPlace
    FeetPlaceXBuffer = msg.YPlace

def FeetOnFloor_cb(msg):
    listener.flag = 1


if __name__ == '__main__':
    rospy.init_node("Camera_terrain_adaptation")
    topic = '/camera/depth/image_raw'
    listener = ImageListener(topic)
    pub = rospy.Publisher('/simple_hexapod/changed_vel_path_var', PathVar_n_cmdVel, queue_size=1)
    subGPS = rospy.Subscriber("/simple_hexapod/fix", NavSatFix, nav_cb, queue_size=1)
    subIMU = rospy.Subscriber("/simple_hexapod/imu", Imu, imu_cb, queue_size=1)
    subWorldFeetPos = rospy.Subscriber("WorldFeetPlace", WorldFeetPlace, FeetPlace_cb)
    FeetOnFloorFlag_sub = rospy.Subscriber("/FeetOnFloorFlag",Float32, FeetOnFloor_cb)
    rospy.sleep(1)

    msg = PathVar_n_cmdVel()

    robot = HexapodC()
    
    H = 300 #height of cam
    L = 0 #X offset of cam
    CA = 40 #cam angle
    LI = 17.5 #Left imager offset
    
    HL = np.sqrt(H**2+L**2) #slope of offsets
    AngHL = np.arctan2(H,L) #Angle of offsets

    while not rospy.is_shutdown():

        FootXPos_world = np.array([450,450,450,600,600,600])
        FootYPos_world = np.array([0,125,-125,0,125,-125])
        FootXPos = FootXPos_world*np.cos(-yaw) - FootYPos_world*np.sin(-yaw) - np.cos(np.arctan2(-e*1000,n*1000)-yaw)*np.sqrt((n*1000)**2+(e*1000)**2)
        FootYPos = FootXPos_world*np.sin(-yaw) + FootYPos_world*np.cos(-yaw) - np.sin(np.arctan2(-e*1000,n*1000)-yaw)*np.sqrt((n*1000)**2+(e*1000)**2)
        
        flag = [0,0,0,0,0,0]
        zmax = [0,0,0,0,0,0]
        jmin = [99999,99999,99999,99999,99999,99999]
        TransZ = np.array([0.0,0.0,0.0,0.0,0.0,0.0])

        yc = [None for i in range(1000+1)]
        zc = [None for i in range(1000+1)]

        i = 0
        for h in range(-200,800+1):
            yc[i] = (np.cos((90+CA)*np.pi/180)*FootXPos - np.sin((90+CA)*np.pi/180)*h + HL*np.sin(AngHL+CA*np.pi/180))
            zc[i] = (np.sin((90+CA)*np.pi/180)*FootXPos + np.cos((90+CA)*np.pi/180)*h - HL*np.cos(AngHL+CA*np.pi/180))
            i = i + 1

        xc = -FootYPos + LI

        for k in range(0,6):
            for i,j in zip(yc,zc):
                if listener.intrinsics:
                    Pixels = rs2.rs2_project_point_to_pixel(listener.intrinsics,[xc[k],i[k],j[k]])
                    Pixels[0] = round(Pixels[0])
                    Pixels[1] = round(Pixels[1])
                    Pixels[0] = 0 if Pixels[0] < 0 else (listener.intrinsics.width-1 if Pixels[0] >= listener.intrinsics.width else Pixels[0])
                    Pixels[1] = 0 if Pixels[1] < 0 else (listener.intrinsics.height-1 if Pixels[1] >= listener.intrinsics.height else Pixels[1])
                    depth = listener.cv_image[Pixels[1],Pixels[0]] 

                    if abs((depth-j[k])/depth) < 0.02:
                        flag[k] = 1
                        z = depth 

                        if j[k] < jmin[k]:
                            jmin[k] = j[k]
                            zmax[k] = z
                            TransZ[k] = round(np.sin((-90-CA)*np.pi/180)*i[k] + np.cos((-90-CA)*np.pi/180)*j[k] + HL*np.sin(AngHL))
                    
            if flag[k] == 0:
                zmax[k] = None
                TransZ[k]= None
        
        #print([zmax,TransZ])

        TransZ_world = np.array([0.0,0.0,0.0,0.0,0.0,0.0])

        beta = 0*np.pi/180; gamma = -0*np.pi/180; alpha = 0; Z_gps = 123

        TransZ_world = (np.cos(beta)*np.sin(gamma)*np.sin(alpha)-np.sin(beta)*np.cos(alpha))*FootXPos + (np.sin(beta)*np.sin(alpha)+np.cos(beta)*np.sin(gamma)*np.cos(alpha))*FootYPos + (np.cos(beta)*np.cos(gamma))*TransZ + Z_gps

        #print(TransZ); 
        print("\t"); print(TransZ_world)

        msg.path_var.Sh = [50, 50, 50, 50, 50, 50]
        msg.path_var.Fh = [0, 0, 0, 0, 0, 0]
        msg.Name = 'Camera'
        pub.publish(msg)
        