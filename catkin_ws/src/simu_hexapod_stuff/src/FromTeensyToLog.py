#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
import time
from FK import FK03_inbody
from gazebo_msgs.srv import GetModelState, GetJointProperties
from numpy import cos,sin
from ToEulerAngles import ToEulerAng
from my_message.msg import WorldFeetPlace
import numpy as np

# def logdata_cb(msg):
#     file.write(msg.data + "\n")

# def logdata2_cb(msg):
#     file2.write(str(time.time()*1000) + "," + str(msg.data[1]) + "\n")
    

# if __name__ == '__main__':

#     rospy.init_node('TeensyLOG')

#     fileName = 'Sent_Datav2.csv'
#     fileName2 = 'Recievedv3_Data.csv'
#     file = open(fileName,"w")
#     file2 = open(fileName2,"w")

#     sub_logdata = rospy.Subscriber('LOGDATA',String,logdata_cb,queue_size=1)
#     sub_logdata2 = rospy.Subscriber('/simple_hexapod/Th2_1_position_controller/command',Float64MultiArray,logdata2_cb,queue_size=1)

#     while not rospy.is_shutdown():
#         rospy.sleep(1)

#     file.close()
#     file2.close()
#     print('done')

#=================================================================================================


flag1 = 0
theta1 = 0
def logdata1_cb(msg):
    global flag1, theta1
    flag1 = 1
    theta1 = msg.data[1]

flag2 = 0
theta2 = 0
def logdata2_cb(msg):
    global flag2, theta2
    flag2 = 1
    theta2 = msg.data[1]

flag3 = 0
theta3 = 0
def logdata3_cb(msg):
    global flag3, theta3
    flag3 = 1
    theta3 = msg.data[1]

flag4 = 0
theta1_3 = 0
def logdata4_cb(msg):
    global flag4, theta1_3
    flag4 = 1
    theta1_3 = msg.data[1]

flag5 = 0
theta2_3 = 0
def logdata5_cb(msg):
    global flag5, theta2_3
    flag5 = 1
    theta2_3 = msg.data[1]

flag6 = 0
theta3_3 = 0
def logdata6_cb(msg):
    global flag6, theta3_3
    flag6 = 1
    theta3_3 = msg.data[1]

flag7 = 0
theta1_5 = 0
def logdata7_cb(msg):
    global flag7, theta1_5
    flag7 = 1
    theta1_5 = msg.data[1]

flag8 = 0
theta2_5 = 0
def logdata8_cb(msg):
    global flag8, theta2_5
    flag8 = 1
    theta2_5 = msg.data[1]

flag9 = 0
theta3_5 = 0
def logdata9_cb(msg):
    global flag9, theta3_5
    flag9 = 1
    theta3_5 = msg.data[1]

FeetPlaceXBuffer =[0]; FeetPlaceYBuffer =[0]
NewPosFlag = -1
def FeetPlace_cb(msg):
    global FeetPlaceXBuffer, FeetPlaceYBuffer, NewPosFlag
    if NewPosFlag == -1:
        NewPosFlag = 1
        FeetPlaceXBuffer = np.array(msg.XPlace)*1000
        FeetPlaceYBuffer = np.array(msg.YPlace)*1000
    elif NewPosFlag == 1:
        NewPosFlag = 0
    



if __name__ == '__main__':

    rospy.init_node('LOGmyData')

    model_coordinates = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)
    joint_prop = rospy.ServiceProxy('/gazebo/get_joint_properties',GetJointProperties)

    fileName = 'DataV5_2.csv'
    file = open(fileName,"w")
    rospy.Subscriber('/simple_hexapod/Th1_2_position_controller/command',Float64MultiArray,logdata1_cb,queue_size=1)
    rospy.Subscriber('/simple_hexapod/Th2_2_position_controller/command',Float64MultiArray,logdata2_cb,queue_size=1)
    rospy.Subscriber('/simple_hexapod/Th3_2_position_controller/command',Float64MultiArray,logdata3_cb,queue_size=1)

    rospy.Subscriber('/simple_hexapod/Th1_3_position_controller/command',Float64MultiArray,logdata4_cb,queue_size=1)
    rospy.Subscriber('/simple_hexapod/Th2_3_position_controller/command',Float64MultiArray,logdata5_cb,queue_size=1)
    rospy.Subscriber('/simple_hexapod/Th3_3_position_controller/command',Float64MultiArray,logdata6_cb,queue_size=1)

    rospy.Subscriber('/simple_hexapod/Th1_1_position_controller/command',Float64MultiArray,logdata7_cb,queue_size=1)
    rospy.Subscriber('/simple_hexapod/Th2_1_position_controller/command',Float64MultiArray,logdata8_cb,queue_size=1)
    rospy.Subscriber('/simple_hexapod/Th3_1_position_controller/command',Float64MultiArray,logdata9_cb,queue_size=1)

    subWorldFeetPos = rospy.Subscriber("WorldFeetPlace", WorldFeetPlace, FeetPlace_cb)

    while not rospy.is_shutdown():
        if flag1 == 1 and flag2 == 1 and flag3 == 1 :
            flag1 = 0;flag2 = 0;flag3 = 0;flag4 = 0;flag5 = 0;flag6 = 0;NewPosFlag = -1

        # joint_positions = joint_prop("Th1_1")
        # theta1 = joint_positions.position[0]
        # joint_positions = joint_prop("Th2_1")
        # theta2 = joint_positions.position[0]
        # joint_positions = joint_prop("Th3_1")
        # theta3 = joint_positions.position[0]

            object_coordinates = model_coordinates("simple_hexapod","")
            bodyXpos = object_coordinates.pose.position.x*1000
            bodyYpos = object_coordinates.pose.position.y*1000
            bodyZpos = object_coordinates.pose.position.z*1000
            (bodyroll,bodypitch,bodyYaw) = ToEulerAng(object_coordinates.pose.orientation.x,object_coordinates.pose.orientation.y,object_coordinates.pose.orientation.z,object_coordinates.pose.orientation.w)

            (Px, Py, Pz) = FK03_inbody(theta1,theta2,theta3,1)
            (Px_3, Py_3, Pz_3) = FK03_inbody(theta1_3,theta2_3,theta3_3,2)
            (Px_5, Py_5, Pz_5) = FK03_inbody(theta1_5,theta2_5,theta3_5,0)

            Px_w = Px*cos(bodyYaw) - Py*sin(bodyYaw) + bodyXpos
            Py_w = Px*sin(bodyYaw) + Py*cos(bodyYaw) + bodyYpos
            Pz_w = Pz + 140#bodyZpos+15

            Px_3_w = Px_3*cos(bodyYaw) - Py_3*sin(bodyYaw) + bodyXpos
            Py_3_w = Px_3*sin(bodyYaw) + Py_3*cos(bodyYaw) + bodyYpos
            Pz_3_w = Pz_3 + 140#bodyZpos+15

            Px_5_w = Px_5*cos(bodyYaw) - Py_5*sin(bodyYaw) + bodyXpos
            Py_5_w = Px_5*sin(bodyYaw) + Py_5*cos(bodyYaw) + bodyYpos
            Pz_5_w = Pz_5 + 140#bodyZpos+15

            file.write(str(time.time()*1000) + "," + str(Pz_w) + "," + str(Pz_3_w) + "," + str(Pz_5_w) + "\n")

        # rospy.sleep(0.01)

    file.close()
    print('done')