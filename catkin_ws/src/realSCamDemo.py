#!/usr/bin/env python

# import rospy
# import cv2
# from cv_bridge import CvBridge, CvBridgeError
# from sensor_msgs.msg import Image
# import numpy as np

# def convert_depth_image(ros_image):
#     bridge = CvBridge()
#      # Use cv_bridge() to convert the ROS image to OpenCV format
#     try:
#      #Convert the depth image using the default passthrough encoding
#         depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
#         depth_array = np.array(depth_image, dtype=np.float32)
#         center_idx = np.array(depth_array.shape) // 2
#         print ('center depth:', depth_array[center_idx[0],center_idx[1]])

#     except CvBridgeError as e:
#         print(e)
#      #Convert the depth image to a Numpy array


# def pixel2depth():
#     rospy.init_node('pixel2depth',anonymous=True)
#     rospy.Subscriber("/camera/depth/image_raw", Image,callback=convert_depth_image, queue_size=1)
#     rospy.spin()

# if __name__ == '__main__':
#     pixel2depth()

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

            if self.intrinsics:

                self.SomeTable = {}

                # for j in range(data.height):
                #     for i in range(data.width):
                #         depth = self.cv_image[j,i]
                #         temp = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [i, j], depth)
                #         self.SomeTable[(round(temp[0]),round(temp[1]))] = round(temp[2])

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
    rospy.init_node("node_name")
    topic = '/camera/depth/image_raw'
    listener = ImageListener(topic)
    rospy.sleep(1)
    
    TransTable = {}
    H = 1000 #height of cam
    CA = 45 #cam angle
    LI = (17.5) #Left imager offset
    FootXPos = 1000
    FootYPos = 0

    while not rospy.is_shutdown():
        
        TableHolder = dict(listener.SomeTable)

        flag = 0

        zmax = 0
        jmin = 99999

        # for i,j in TableHolder.items():
        #     if i[0] != 0 and i[1] !=0 and j != 0:
        #         TransX = -0.7071*i[1]+0.7071*j
        #         TransY = -i[0]+17.5
        #         TransZ = -0.7071*i[1]-0.7071*j+1000

        #         TransTable[TransX,TransY] = TransZ

        yc = [None for i in range(1000+1)]
        zc = [None for i in range(1000+1)]

        for h in range(0,1000+1):
            yc[h] = (-0.7071*FootXPos-0.7071*h + H*np.sin((90 - CA)*np.pi/180))
            zc[h] = (0.7071*FootXPos-0.7071*h + H*np.cos((90 - CA)*np.pi/180))

        xc = -FootYPos + LI

        for i,j in zip(yc,zc):
            # Z = listener.SomeTable.get((xc,i))
            # if Z != None:
            #     if abs((Z-j)/Z) < 0.01:
            #         print(Z)

            if listener.intrinsics:

                Pixels = rs2.rs2_project_point_to_pixel(listener.intrinsics,[xc,i,j])
                Pixels[1] = 0 if Pixels[1] < 0 else Pixels[1]
                depth = listener.cv_image[round(Pixels[1]),round(Pixels[0])] 
                if abs((depth-j)/depth) < 0.02:
                    flag = 1
                    z = depth
                    
                    if j < jmin:
                        jmin = j
                        zmax = z
                        TransZ = -0.7071*i-0.7071*j+1000
                    
        if flag == 0:
            zmax = None
            TransZ = None
        
        print([zmax,TransZ])