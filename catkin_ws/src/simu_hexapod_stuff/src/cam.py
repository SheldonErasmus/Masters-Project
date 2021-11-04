#!/usr/bin/env python3
#Using the acquired binocular image for positioning

import rospy
import cv2
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import math #Calculate Tangent
from matplotlib import pyplot as plt
import numpy as np

bridge = CvBridge()
hfov = 1.3962634
image_width = 800
flength = (image_width/2) / ( math.tan(hfov/2) )
base_line = 0.2
cx = 400
cy = 400

def callback(left_ros_image,right_ros_image):
    global flength
    global cx 
    global cy

    left_cv_image = bridge.imgmsg_to_cv2(left_ros_image,"mono8")
    right_cv_image = bridge.imgmsg_to_cv2(right_ros_image, "mono8")
   
   #Image display
    cv2.imshow('left_image',left_cv_image)
    cv2.imshow('right_image',right_cv_image)

    stereo = cv2.StereoBM_create()

    numDisparities = cv2.getTrackbarPos('numDisparities','disp')*16
    blockSize = cv2.getTrackbarPos('blockSize','disp')*2 + 5
    preFilterType = cv2.getTrackbarPos('preFilterType','disp')
    preFilterSize = cv2.getTrackbarPos('preFilterSize','disp')*2 + 5
    preFilterCap = cv2.getTrackbarPos('preFilterCap','disp')
    textureThreshold = cv2.getTrackbarPos('textureThreshold','disp')
    uniquenessRatio = cv2.getTrackbarPos('uniquenessRatio','disp')
    speckleRange = cv2.getTrackbarPos('speckleRange','disp')
    speckleWindowSize = cv2.getTrackbarPos('speckleWindowSize','disp')*2
    disp12MaxDiff = cv2.getTrackbarPos('disp12MaxDiff','disp')
    minDisparity = cv2.getTrackbarPos('minDisparity','disp')

    stereo.setNumDisparities(numDisparities)
    stereo.setBlockSize(blockSize)
    stereo.setPreFilterType(preFilterType)
    stereo.setPreFilterSize(preFilterSize)
    stereo.setPreFilterCap(preFilterCap)
    stereo.setTextureThreshold(textureThreshold)
    stereo.setUniquenessRatio(uniquenessRatio)
    stereo.setSpeckleRange(speckleRange)
    stereo.setSpeckleWindowSize(speckleWindowSize)
    stereo.setDisp12MaxDiff(disp12MaxDiff)
    stereo.setMinDisparity(minDisparity)


    disparity = stereo.compute(left_cv_image,right_cv_image)
    disparity = cv2.normalize(disparity, None, alpha = 0, beta = 1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
    # disparity = disparity.astype(np.float32)

    # # Scaling down the disparity values and normalizing them
    # disparity = (disparity/16.0 - minDisparity)/numDisparities

    cv2.imshow('disp', disparity)
    cv2.waitKey(3)

def nothing(x):
    pass

if __name__ == '__main__':
    rospy.init_node('gazebo_image_sub', anonymous=True)

    cv2.namedWindow('disp',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('disp',800,800)

    cv2.createTrackbar('numDisparities','disp',3,100,nothing)
    cv2.createTrackbar('blockSize','disp',5,50,nothing)
    cv2.createTrackbar('preFilterType','disp',0,1,nothing)
    cv2.createTrackbar('preFilterSize','disp',1,25,nothing)
    cv2.createTrackbar('preFilterCap','disp',7,62,nothing)
    cv2.createTrackbar('textureThreshold','disp',7,100,nothing)
    cv2.createTrackbar('uniquenessRatio','disp',12,100,nothing)
    cv2.createTrackbar('speckleRange','disp',1,100,nothing)
    cv2.createTrackbar('speckleWindowSize','disp',3,25,nothing)
    cv2.createTrackbar('disp12MaxDiff','disp',2,25,nothing)
    cv2.createTrackbar('minDisparity','disp',4,25,nothing)

    left_ros_image = message_filters.Subscriber("/stereocamera/left/image_raw", Image)
    right_ros_image =message_filters.Subscriber("/stereocamera/right/image_raw", Image)
    ts = message_filters.TimeSynchronizer([left_ros_image , right_ros_image], 10)
    ts.registerCallback(callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()