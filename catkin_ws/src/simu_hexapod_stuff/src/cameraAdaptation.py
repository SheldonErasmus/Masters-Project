#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo, NavSatFix, Imu
from gazebo_msgs.srv import GetModelState
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
        self.cv_image = [np.ones((720,1280)) for i in range(5)]
        self.flag = 0

        pose = {"x":0,"y":0, "z":0,"yaw":0}
        self.image_pose = [pose for i in range(5)]

        self.SomeTable = {}

    def imageDepthCallback(self, data):
        if self.flag == 1:
            self.flag = 0 
            try:
                temp = [self.bridge.imgmsg_to_cv2(data, data.encoding)]
                self.cv_image = np.concatenate((self.cv_image[1:5],temp))
                pose = [{"x":n,"y":e, "z":d,"yaw":yaw}]
                self.image_pose = np.concatenate((self.image_pose[1:5],pose))
                self.SomeTable = {}
            except CvBridgeError as er:
                print(er)
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
    object_coordinates = model_coordinates("simple_hexapod","")
    n = object_coordinates.pose.position.x
    e = -object_coordinates.pose.position.y
    d = -object_coordinates.pose.position.z

roll=0;pitch=0;yaw=0
def imu_cb(msg):
    global roll,pitch,yaw
    (roll,pitch,yaw) = ToEulerAng(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)

FeetPlaceXBuffer = np.ones(12)/1000
FeetPlaceYBuffer = np.ones(12)/1000
NewPosFlag = 0
adjustHeightFlag = -1
def FeetPlace_cb(msg):
    global FeetPlaceXBuffer, FeetPlaceYBuffer, NewPosFlag,adjustHeightFlag
    FeetPlaceXBuffer = np.concatenate((FeetPlaceXBuffer[6:12],msg.XPlace))
    FeetPlaceYBuffer = np.concatenate((FeetPlaceYBuffer[6:12],msg.YPlace))
    NewPosFlag = 1
    adjustHeightFlag = 0

def FeetOnFloor_cb(msg):
    global adjustHeightFlag
    listener.flag = 1
    if adjustHeightFlag == -1: adjustHeightFlag = 1


if __name__ == '__main__':
    rospy.init_node("Camera_terrain_adaptation")
    model_coordinates = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)
    topic = '/camera/depth/image_raw'
    listener = ImageListener(topic)
    pub = rospy.Publisher('/simple_hexapod/changed_vel_path_var', PathVar_n_cmdVel, queue_size=1)
    subGPS = rospy.Subscriber("/simple_hexapod/fix", NavSatFix, nav_cb, queue_size=1)
    subIMU = rospy.Subscriber("/simple_hexapod/imu", Imu, imu_cb, queue_size=1)
    subWorldFeetPos = rospy.Subscriber("WorldFeetPlace", WorldFeetPlace, FeetPlace_cb)
    FeetOnFloorFlag_sub = rospy.Subscriber("/FeetOnFloorFlag",Float32, FeetOnFloor_cb)
    rospy.sleep(1)
    listener.flag = 1
    msg = PathVar_n_cmdVel(); msg.path_var.Fh = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])

    robot = HexapodC()
    
    H = 300 #height of cam
    L = 0 #X offset of cam
    CA = 40 #cam angle
    LI = 17.5 #Left imager offset
    
    HL = np.sqrt(H**2+L**2) #slope of offsets
    AngHL = np.arctan2(H,L) #Angle of offsets

    TransZ_world = np.array([0.0,0.0,0.0,0.0,0.0,0.0])

    while not rospy.is_shutdown():

        FootXPos_world = FeetPlaceXBuffer[0:6]*1000 
        #FootXPos_world = np.array([450.0,450.0,450.0,600.0,600.0,600.0])
        FootYPos_world = FeetPlaceYBuffer[0:6]*1000 
        #FootYPos_world = np.array([0.0,125.0,-125.0,0.0,125.0,-125.0])

        if NewPosFlag == 1:
            NewPosFlag = 0

            flag = [0,0,0,0,0,0]
            zmax = [0,0,0,0,0,0]
            jmin = [99999,99999,99999,99999,99999,99999]
            TransZ = np.array([0.0,0.0,0.0,0.0,0.0,0.0])

            for k in range(0,6):
                loopCounter = 0; loopBreak = 0
                while loopCounter <= 4 and loopBreak == 0:
                    
                    n_snapshot = round(listener.image_pose[loopCounter]["x"],3)
                    e_snapshot = round(listener.image_pose[loopCounter]["y"],3)
                    d_snapshot = round(listener.image_pose[loopCounter]["z"],3)
                    yaw_snapshot = listener.image_pose[loopCounter]["yaw"]

                    FootXPos = FootXPos_world[k]*np.cos(-yaw_snapshot) - FootYPos_world[k]*np.sin(-yaw_snapshot) - np.cos(np.arctan2(-e_snapshot*1000,n_snapshot*1000)-yaw_snapshot)*np.sqrt((n_snapshot*1000)**2+(e_snapshot*1000)**2)
                    FootYPos = FootXPos_world[k]*np.sin(-yaw_snapshot) + FootYPos_world[k]*np.cos(-yaw_snapshot) - np.sin(np.arctan2(-e_snapshot*1000,n_snapshot*1000)-yaw_snapshot)*np.sqrt((n_snapshot*1000)**2+(e_snapshot*1000)**2)
                    
                    

                    yc = [None for i in range(1000+1)]
                    zc = [None for i in range(1000+1)]

                    i = 0
                    for h in range(-200,800+1):
                        yc[i] = (np.cos((90+CA)*np.pi/180)*FootXPos - np.sin((90+CA)*np.pi/180)*h + HL*np.sin(AngHL+CA*np.pi/180))
                        zc[i] = (np.sin((90+CA)*np.pi/180)*FootXPos + np.cos((90+CA)*np.pi/180)*h - HL*np.cos(AngHL+CA*np.pi/180))
                        i = i + 1

                    if k == 3:
                        k3_count = 0
                        TransZ_world_temp = [0.0,0.0]
                        FootYPos = FootYPos + np.array([-20,20])
                        xc = -FootYPos + LI
                    else:
                        xc = [None]
                        xc[0] = -FootYPos + LI

                    
                    for XC in xc:
                        for i,j in zip(yc,zc):
                            if listener.intrinsics:
                                Pixels = rs2.rs2_project_point_to_pixel(listener.intrinsics,[XC,i,j])
                                Pixels[0] = round(Pixels[0])
                                Pixels[1] = round(Pixels[1])
                                #Pixels[0] = 0 if Pixels[0] < 0 else (listener.intrinsics.width-1 if Pixels[0] >= listener.intrinsics.width else Pixels[0])
                                #Pixels[1] = 0 if Pixels[1] < 0 else (listener.intrinsics.height-1 if Pixels[1] >= listener.intrinsics.height else Pixels[1])
                                if Pixels[0] < 0 or Pixels[0] >= listener.intrinsics.width: continue
                                if Pixels[1] < 0 or Pixels[1] >= listener.intrinsics.height: continue
                                depth = listener.cv_image[loopCounter][Pixels[1],Pixels[0]] 

                                if abs((depth-j)/depth) < 0.02:
                                    loopBreak = 1
                                    flag[k] = 1
                                    z = depth 

                                    if j < jmin[k]:
                                        jmin[k] = j
                                        zmax[k] = z
                                        TransZ[k] = round(np.sin((-90-CA)*np.pi/180)*i + np.cos((-90-CA)*np.pi/180)*j + HL*np.sin(AngHL))
                            
                        if flag[k] == 0:
                            zmax[k] = None
                            TransZ[k]= None
                    
                        #print([zmax,TransZ])

                        beta = 0*np.pi/180; gamma = -0*np.pi/180; alpha = 0; Z_gps = -d_snapshot*1000

                        if k == 3:
                            jmin[k] = 99999
                            TransZ_world_temp[k3_count] = (np.cos(beta)*np.sin(gamma)*np.sin(alpha)-np.sin(beta)*np.cos(alpha))*FootXPos + (np.sin(beta)*np.sin(alpha)+np.cos(beta)*np.sin(gamma)*np.cos(alpha))*FootYPos[k3_count] + (np.cos(beta)*np.cos(gamma))*TransZ[k] + Z_gps
                            if k3_count == 1:
                                TransZ_world[k] = (TransZ_world_temp[0] + TransZ_world_temp[1])/2
                            k3_count = k3_count + 1
                        else:
                            TransZ_world[k] = (np.cos(beta)*np.sin(gamma)*np.sin(alpha)-np.sin(beta)*np.cos(alpha))*FootXPos + (np.sin(beta)*np.sin(alpha)+np.cos(beta)*np.sin(gamma)*np.cos(alpha))*FootYPos + (np.cos(beta)*np.cos(gamma))*TransZ[k] + Z_gps

                    loopCounter = loopCounter + 1

            #print(TransZ); 
            print("\t"); print(TransZ_world)
            
        if adjustHeightFlag == 0:
            print(adjustHeightFlag)
            adjustHeightFlag = -1

            for k in [1,3,5]:
                    msg.path_var.Fh[k] = msg.path_var.Fh[k+6]

            for k in [0,2,4]:
                if np.isnan(TransZ_world[k]):
                    msg.path_var.Fh[k+6] = msg.path_var.Fh[k+6]
                else:
                    msg.path_var.Fh[k+6] = TransZ_world[k]

            for k in [0,2,4]:
                if msg.path_var.Fh[k+6] > msg.path_var.Fh[k]:
                    msg.path_var.Fh[k] = msg.path_var.Fh[k+6]

            msg.path_var.Sh = [50, 50, 50, 50, 50, 50]+msg.path_var.Fh[0:6]
            #msg.path_var.Fh = TransZ_world[0:6]
            msg.Name = 'Camera'
            pub.publish(msg)
        elif adjustHeightFlag == 1:
            print(adjustHeightFlag)
            adjustHeightFlag = -2

            for k in [0,2,4]:
                    msg.path_var.Fh[k] = msg.path_var.Fh[k+6]

            for k in [1,3,5]:
                if np.isnan(TransZ_world[k]):
                    msg.path_var.Fh[k+6] = msg.path_var.Fh[k+6]
                else:
                    msg.path_var.Fh[k+6] = TransZ_world[k]

            for k in [1,3,5]:
                if msg.path_var.Fh[k+6] > msg.path_var.Fh[k]:
                    msg.path_var.Fh[k] = msg.path_var.Fh[k+6]

            msg.path_var.Sh = [50, 50, 50, 50, 50, 50]+msg.path_var.Fh[0:6]
            #msg.path_var.Fh = TransZ_world[0:6]
            msg.Name = 'Camera'
            pub.publish(msg)

        
        