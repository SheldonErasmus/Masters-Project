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
        self.sub = rospy.Subscriber(self.topic, PointCloud2, self.PCCallback)
        self.sub1 = rospy.Subscriber('/camera/depth/image_raw', msg_Image, self.imageDepthCallback)

    def PCCallback(self, data):
        pc = ros_numpy.numpify(data)
        self.pointsX = np.round(pc['x']*1000)
        self.pointsY = np.round(pc['y']*1000)
        self.pointsZ = np.round(pc['z']*1000)

        self.PointsX_H = np.cos((-90-CA)*np.pi/180)*self.pointsY - np.sin((-90-CA)*np.pi/180)*self.pointsZ + HL*np.cos(AngHL)
        self.PointsY_H = -self.pointsX + LI
        self.PointsZ_H = np.sin((-90-CA)*np.pi/180)*self.pointsY + np.cos((-90-CA)*np.pi/180)*self.pointsZ + HL*np.sin(AngHL)
        weee = 0

    def imageDepthCallback(self,data):
        cv_img = self.bridge.imgmsg_to_cv2(data, data.encoding)
        self.img = ros_numpy.numpify(data)
        weee = 0

H = 300 #height of cam
L = 0 #X offset of cam
CA = 40 #cam angle
LI = 17.5 #Left imager offset

HL = np.sqrt(H**2+L**2) #slope of offsets
AngHL = np.arctan2(H,L) #Angle of offsets

FootX_H = 500
FootY_H = 0

if __name__ == '__main__':
    rospy.init_node("Camera_terrain_adaptation")
    model_coordinates = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)
    topic = '/camera/depth/color/points'
    listener = ImageListener(topic)
    rospy.sleep(1)

    while not rospy.is_shutdown():

        indexX = np.unravel_index(np.argmin(np.absolute(listener.PointsX_H-FootX_H)),listener.PointsX_H.shape)
        indexY = np.argmin(np.absolute(listener.PointsY_H[indexX[0]]-FootY_H))
        print(indexX[0],indexY,listener.PointsZ_H[indexX[0]][indexY])
        #print(listener.PointsZ_H[0][0])