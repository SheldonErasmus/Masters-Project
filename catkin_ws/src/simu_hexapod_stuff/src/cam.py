#!/usr/bin/env python3

import numpy as np
import rospy
import ros_numpy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo, NavSatFix, Imu, PointCloud2
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

        self.flag = 0
        self.pc = [np.ones((720,1280),dtype={'names':['x','y','z','rgb'], 'formats':['<f4','<f4','<f4','<f4'], 'offsets':[0,4,8,16], 'itemsize':32}) for i in range(6)]
        pose = {"x":0,"y":0, "z":0,"yaw":0}
        self.image_pose = [pose for i in range(6)]
        self.sub = rospy.Subscriber(self.topic, PointCloud2, self.PCCallback)
        

    def PCCallback(self, data):
        if self.flag == 1:
            self.flag = 0

            temp = [ros_numpy.numpify(data)]
            self.pc = np.concatenate((self.pc[1:6],temp))
            pose = [{"x":n,"y":e, "z":d,"yaw":yaw}]
            self.image_pose = np.concatenate((self.image_pose[1:6],pose))
        weee = 0

n = 0.0
e = 0.0
d = 0.0
def nav_cb(msg):
    global n, e, d
    object_coordinates = model_coordinates("simple_hexapod","")
    n = object_coordinates.pose.position.x
    e = -object_coordinates.pose.position.y
    d = -object_coordinates.pose.position.z

roll=0;pitch=0;yaw=0
def imu_cb(msg):
    global roll,pitch,yaw
    (roll,pitch,yaw) = ToEulerAng(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)

NewPosFlag = 0
adjustHeightFlag = -2
def FeetPlace_cb(msg):
    global FeetPlaceXBuffer, FeetPlaceYBuffer, NewPosFlag,adjustHeightFlag
    FeetPlaceXBuffer = np.array(msg.XPlace) 
    FeetPlaceYBuffer = np.array(msg.YPlace) 
    NewPosFlag = 1
    if adjustHeightFlag == -2: adjustHeightFlag = 0
    if adjustHeightFlag == -1: adjustHeightFlag = 1

def FeetOnFloor_cb(msg):
    global adjustHeightFlag
    listener.flag = 1


H = 300 #height of cam
L = 12 #X offset of cam
CA = 40 #cam angle
LI = 17.5 #Left imager offset

HL = np.sqrt(H**2+L**2) #slope of offsets
AngHL = np.arctan2(H,L) #Angle of offsets

FootX_w = [1000.0,450.0,450.0,600.0,600.0,600.0]
FootY_w = [0.0,125.0,-125.0,0.0,125.0,-125.0]

if __name__ == '__main__':
    rospy.init_node("Camera_terrain_adaptation")
    model_coordinates = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)
    topic = '/camera/depth/color/points'
    listener = ImageListener(topic)
    pub = rospy.Publisher('/simple_hexapod/changed_vel_path_var', PathVar_n_cmdVel, queue_size=1)
    subGPS = rospy.Subscriber("/simple_hexapod/fix", NavSatFix, nav_cb, queue_size=1)
    subIMU = rospy.Subscriber("/simple_hexapod/imu", Imu, imu_cb, queue_size=1)
    subWorldFeetPos = rospy.Subscriber("WorldFeetPlace", WorldFeetPlace, FeetPlace_cb)
    FeetOnFloorFlag_sub = rospy.Subscriber("/FeetOnFloorFlag",Float32, FeetOnFloor_cb)
    rospy.sleep(1)
    listener.flag = 1
    msg = PathVar_n_cmdVel(); msg.path_var.Fh = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])

    rospy.wait_for_message('WorldFeetPlace',WorldFeetPlace)

    TransZ_world = np.array([0.0,0.0,0.0,0.0,0.0,0.0])

    while not rospy.is_shutdown():

        FootXPos_world = FeetPlaceXBuffer*1000
        FootYPos_world = FeetPlaceYBuffer*1000 

        if NewPosFlag == 1:
            NewPosFlag = 0

            TransZ = np.array([0.0,0.0,0.0,0.0,0.0,0.0])

            for k in range(0,6):
                loopCounter = 0; loopBreak = 0
                while loopCounter <= 5 and loopBreak == 0:

                    n_snapshot = round(listener.image_pose[loopCounter]["x"],3)
                    e_snapshot = round(listener.image_pose[loopCounter]["y"],3)
                    d_snapshot = round(listener.image_pose[loopCounter]["z"],3)
                    yaw_snapshot = listener.image_pose[loopCounter]["yaw"]

                    pointsX = listener.pc[loopCounter]['x']*1000
                    pointsY = listener.pc[loopCounter]['y']*1000
                    pointsZ = listener.pc[loopCounter]['z']*1000

                    PointsX_H = np.round(np.cos((-90-CA)*np.pi/180)*pointsY - np.sin((-90-CA)*np.pi/180)*pointsZ + HL*np.cos(AngHL))
                    PointsY_H = np.round(-pointsX + LI)
                    PointsZ_H = np.round(np.sin((-90-CA)*np.pi/180)*pointsY + np.cos((-90-CA)*np.pi/180)*pointsZ + HL*np.sin(AngHL))

                    PointsX_W = np.round(np.cos(yaw_snapshot)*PointsX_H - np.sin(yaw_snapshot)*PointsY_H + n_snapshot*1000)
                    PointsY_W = np.round(np.sin(yaw_snapshot)*PointsX_H + np.cos(yaw_snapshot)*PointsY_H - e_snapshot*1000)
                    PointsZ_W = np.round(PointsZ_H - d_snapshot*1000 + 10)

                    FootXPos_H = np.round(FootXPos_world[k]*np.cos(-yaw_snapshot) - FootYPos_world[k]*np.sin(-yaw_snapshot) - np.cos(np.arctan2(-e_snapshot*1000,n_snapshot*1000)-yaw_snapshot)*np.sqrt((n_snapshot*1000)**2+(e_snapshot*1000)**2))
                    FootYPos_H = np.round(FootXPos_world[k]*np.sin(-yaw_snapshot) + FootYPos_world[k]*np.cos(-yaw_snapshot) - np.sin(np.arctan2(-e_snapshot*1000,n_snapshot*1000)-yaw_snapshot)*np.sqrt((n_snapshot*1000)**2+(e_snapshot*1000)**2))

                    if k == 3:
                        k3_count = 0
                        TransZ_world_temp = [0.0,0.0]
                        footYPos_H = FootYPos_H + np.array([-20,20])
                    else:
                        footYPos_H = [FootYPos_H]

                    for footY in footYPos_H:
                        if np.any(np.absolute(PointsX_H-FootXPos_H) < 5):
                            indexX = np.unravel_index(np.argmin(np.absolute(PointsX_H-FootXPos_H)),PointsX_H.shape)
                            
                            if np.any(np.absolute(PointsY_H[indexX[0]]-footY) < 5):
                                loopBreak = 1
                                indexY = np.argmin(np.absolute(PointsY_H[indexX[0]]-footY))

                                if k == 3:
                                    TransZ_world_temp[k3_count] = PointsZ_W[indexX[0]][indexY]
                                    if k3_count == 1:
                                        TransZ_world[k] = (TransZ_world_temp[0] + TransZ_world_temp[1])/2
                                    k3_count = k3_count + 1
                                else:
                                    TransZ_world[k] = PointsZ_W[indexX[0]][indexY]

                                #print(indexX[0],indexY,TransZ_world[k])
                            else:
                                TransZ_world[k] = None
                                #print(TransZ_world[k])
                        else:
                            TransZ_world[k] = None
                            #print(TransZ_world[k])
                    
                    #print(listener.PointsZ_H[0][0])

                    loopCounter = loopCounter + 1

            print(TransZ_world)

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