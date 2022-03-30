#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
import cv2
from cv_bridge import CvBridge, CvBridgeError
import pyrealsense2 as rs2

class ImageListener:
    def __init__(self, topic):
        self.topic = topic
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic, msg_Image, self.imageDepthCallback)
        self.sub_info = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.imageDepthInfoCallback)
        self.intrinsics = None
        self.SomeTable = {}

    def imageDepthCallback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
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

if __name__ == '__main__':
    rospy.init_node("Camera_terrain_adaptation")
    topic = '/camera/depth/image_raw'
    listener = ImageListener(topic)
    rospy.sleep(1)
    
    H = 300 #height of cam
    L = 0 #X offset of cam
    CA = 40 #cam angle
    LI = 17.5 #Left imager offset
    FootXPos = 400
    FootYPos = 0

    HL = np.sqrt(H**2+L**2) #slope of offsets
    AngHL = np.arctan2(H,L) #Angle of offsets

    while not rospy.is_shutdown():
        
        flag = 0
        zmax = 0
        jmin = 99999

        yc = [None for i in range(1000+1)]
        zc = [None for i in range(1000+1)]

        for h in range(-200,800+1):
            yc[h] = (np.cos((90+CA)*np.pi/180)*FootXPos - np.sin((90+CA)*np.pi/180)*h + HL*np.sin(AngHL+CA*np.pi/180))
            zc[h] = (np.sin((90+CA)*np.pi/180)*FootXPos + np.cos((90+CA)*np.pi/180)*h - HL*np.cos(AngHL+CA*np.pi/180))

        xc = -FootYPos + LI

        for i,j in zip(yc,zc):
            if listener.intrinsics:
                Pixels = rs2.rs2_project_point_to_pixel(listener.intrinsics,[xc,i,j])
                Pixels[0] = 0 if Pixels[0] < 0 else (listener.intrinsics.width-1 if Pixels[0] >= listener.intrinsics.width else Pixels[0])
                Pixels[1] = 0 if Pixels[1] < 0 else (listener.intrinsics.height-1 if Pixels[1] >= listener.intrinsics.height else Pixels[1])
                depth = listener.cv_image[round(Pixels[1]),round(Pixels[0])] 

                if abs((depth-j)/depth) < 0.02:
                    flag = 1
                    z = depth 

                    if j < jmin:
                        jmin = j
                        zmax = z
                        TransZ = np.sin((-90-CA)*np.pi/180)*i + np.cos((-90-CA)*np.pi/180)*j + HL*np.sin(AngHL)
                    
        if flag == 0:
            zmax = None
            TransZ = None
        
        print([zmax,TransZ])