#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray, Float32
from std_msgs.msg import String
import time
#from FK import FK03_inbody
from gazebo_msgs.srv import GetModelState, GetJointProperties
from numpy import cos,sin
from ToEulerAngles import ToEulerAng
from my_message.msg import WorldFeetPlace
import numpy as np
from re import findall
from marvelmind_nav.msg import hedge_pos_ang, marvelmind_waypoint


class FK:
    def __init__(self):
        pass

    def FK03(self,theta1,theta2,theta3):

        # Link lengths as defined in Figure 5.3
        L1x = 53.17
        L1z = 8
        L2  = 101.88
        L3  = 149.16

        # Transformation from servo angles to end effector positions as given in
        # Equation 5.30
        C1  = cos(theta1)
        S1  = sin(theta1)
        C2  = cos(theta2)
        S2  = sin(theta2)
        C23 = cos(theta2 + theta3)
        S23 = sin(theta2 + theta3)

        T03 = np.array([[C1*C23, -C1*S23,  S1, C1*(L1x + L3*C23 + L2*C2)],
                    [S1*C23, -S1*S23, -C1, S1*(L1x + L3*C23 + L2*C2)],
                    [S23,     C23,   0, L1z + L3*S23 + L2*S2],
                    [0,       0,   0, 1]])
    
        Px = T03[0,3]
        Py = T03[1,3]
        Pz = T03[2,3]

        return Px, Py, Pz

    def FK03_inbody(self,theta1,theta2,theta3,leg):

        x,y,z = self.FK03(theta1,theta2,theta3)

        B = 125.54
        rot = [0*np.pi/180, 60*np.pi/180, 120*np.pi/180, 180*np.pi/180, 240*np.pi/180, 300*np.pi/180]
        
        Px = y*sin(rot[leg])+(x+B)*cos(rot[leg])
        Py = y*cos(rot[leg])-(x+B)*sin(rot[leg])
        Pz = z

        return Px, Py, Pz

n_prev = 0.0
e_prev = 0.0
d_prev = 0.0
curr_Head_hex = 0.0; hex_roll = 0.0; hex_pitch = 0.0

n_cur=0; e_cur=0; d_cur=0; Yaw_meas_pair=0; Flag=0; counter = 98
index=0; total_items=0; movement_type=0; Epoints=0; Npoints=0; DoWaypointFlag=0

def hedge_pos_ang_callback(msg):
    global n_prev, e_prev, d_prev, n_cur, e_cur, d_cur, Yaw_meas_pair, Flag, counter

    e_cur = msg.x_m #*1000
    n_cur = msg.y_m #*1000
    d_cur = -msg.z_m #*1000

    Flag = msg.flags
    #print(n_cur, e_cur, d_cur)

Cor_yaw_cur = 0
def yaw_cb(msg):
    global Cor_yaw_cur
    Cor_yaw_cur = msg.data


flag1 = 0
theta1_5_Send = 0
theta1_5_fb = 0
def logdata1_cb(msg):
    global flag1, theta1_5_Send, theta1_5_fb
    flag1 = 1
    data=[float(s) for s in findall('[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?',msg.data)]
    theta1_5_Send = data[0]
    #theta1_5_fb = data[2]

flag2 = 0
theta2_5_Send = 0
theta2_5_fb = 0
def logdata2_cb(msg):
    global flag2, theta2_5_Send, theta2_5_fb
    flag2 = 1
    data=[float(s) for s in findall('[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?',msg.data)]
    theta2_5_Send = data[0]
    #theta2_5_fb = data[2]

flag3 = 0
theta3_5_Send = 0
theta3_5_fb = 0
def logdata3_cb(msg):
    global flag3, theta3_5_Send, theta3_5_fb
    flag3 = 1
    data=[float(s) for s in findall('[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?',msg.data)]
    theta3_5_Send = data[0]
    #theta3_5_fb = data[2]

flag4 = 0
theta1_3_Send = 0
theta1_3_fb = 0
def logdata4_cb(msg):
    global flag4, theta1_3_Send, theta1_3_fb
    flag4 = 1
    data=[float(s) for s in findall('[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?',msg.data)]
    theta1_3_Send = data[0]
    #theta1_3_fb = data[2]

flag5 = 0
theta2_3_Send = 0
theta2_3_fb = 0
def logdata5_cb(msg):
    global flag5, theta2_3_Send, theta2_3_fb
    flag5 = 1
    data=[float(s) for s in findall('[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?',msg.data)]
    theta2_3_Send = data[0]
    #theta2_3_fb = data[2]

flag6 = 0
theta3_3_Send = 0
theta3_3_fb = 0
def logdata6_cb(msg):
    global flag6, theta3_3_Send, theta3_3_fb
    flag6 = 1
    data=[float(s) for s in findall('[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?',msg.data)]
    theta3_3_Send = data[0]
    #theta3_3_fb = data[2]

flag7 = 0
theta1_1_Send = 0
theta1_1_fb = 0
ttime = 0
def logdata7_cb(msg):
    global flag7, ttime, theta1_1_Send, theta1_1_fb
    flag7 = 1
    data=[float(s) for s in findall('[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?',msg.data)]
    #ttime = data[0]
    theta1_1_Send = data[0]
    #theta1_1_fb = data[2]

flag8 = 0
theta2_1_Send = 0
theta2_1_fb = 0
def logdata8_cb(msg):
    global flag8, theta2_1_Send, theta2_1_fb
    flag8 = 1
    data=[float(s) for s in findall('[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?',msg.data)]
    theta2_1_Send = data[0]
    #theta2_1_fb = data[2]

flag9 = 0
theta3_1_Send = 0
theta3_1_fb = 0
def logdata9_cb(msg):
    global flag9, theta3_1_Send, theta3_1_fb
    flag9 = 1
    data=[float(s) for s in findall('[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?',msg.data)]
    theta3_1_Send = data[0]
    #theta3_1_fb = data[2]

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

    fk = FK()

    fileName = 'dataRec3.csv'
    # file = open(fileName,"w")
    subIndoorGPS = rospy.Subscriber("hedge_pos_ang", hedge_pos_ang, hedge_pos_ang_callback, queue_size=1)
    subyaw = rospy.Subscriber('/simple_hexapod/calculated_yaw', Float32, yaw_cb, queue_size=1)

    rospy.Subscriber('/motor_feedback15',String,logdata1_cb,queue_size=1)
    rospy.Subscriber('/motor_feedback25',String,logdata2_cb,queue_size=1)
    rospy.Subscriber('/motor_feedback35',String,logdata3_cb,queue_size=1)

    rospy.Subscriber('/motor_feedback13',String,logdata4_cb,queue_size=1)
    rospy.Subscriber('/motor_feedback23',String,logdata5_cb,queue_size=1)
    rospy.Subscriber('/motor_feedback33',String,logdata6_cb,queue_size=1)

    rospy.Subscriber('/motor_feedback11',String,logdata7_cb,queue_size=1)
    rospy.Subscriber('/motor_feedback21',String,logdata8_cb,queue_size=1)
    rospy.Subscriber('/motor_feedback31',String,logdata9_cb,queue_size=1)

    subWorldFeetPos = rospy.Subscriber("WorldFeetPlace", WorldFeetPlace, FeetPlace_cb)

    try:
        with open(fileName,"w") as file:
            while not rospy.is_shutdown():
                if flag1 == 1 and flag4 == 1 and flag7 == 1 and NewPosFlag == 0:
                    flag1 = 0;flag2 = 0;flag3 = 0;flag4 = 0;flag5 = 0;flag6 = 0;flag7 = 0;NewPosFlag = -1

                    bodyXpos = e_cur*1000
                    bodyYpos = n_cur*1000
                    # bodyZpos = object_coordinates.pose.position.z*1000
                    bodyYaw = Cor_yaw_cur*np.pi/180


                    (Px_5_Send, Py_5_Send, Pz_5_Send) = fk.FK03_inbody(theta1_5_Send,theta2_5_Send,theta3_5_Send,4)
                    (Px_3_Send, Py_3_Send, Pz_3_Send) = fk.FK03_inbody(theta1_3_Send,theta2_3_Send,theta3_3_Send,2)
                    (Px_1_Send, Py_1_Send, Pz_1_Send) = fk.FK03_inbody(theta1_1_Send,theta2_1_Send,theta3_1_Send,0)

                    (Px_5_fb, Py_5_fb, Pz_5_fb) = fk.FK03_inbody(theta1_5_fb,theta2_5_fb,theta3_5_fb,4)
                    (Px_3_fb, Py_3_fb, Pz_3_fb) = fk.FK03_inbody(theta1_3_fb,theta2_3_fb,theta3_3_fb,2)
                    (Px_1_fb, Py_1_fb, Pz_1_fb) = fk.FK03_inbody(theta1_1_fb,theta2_1_fb,theta3_1_fb,0)

                    Px_5_w = Px_5_Send*cos(bodyYaw) - Py_5_Send*sin(bodyYaw) + bodyXpos
                    Py_5_w = Px_5_Send*sin(bodyYaw) + Py_5_Send*cos(bodyYaw) + bodyYpos
                    # Pz_5_w = Pz_5_Send + 140#bodyZpos+15

                    Px_3_w = Px_3_Send*cos(bodyYaw) - Py_3_Send*sin(bodyYaw) + bodyXpos
                    Py_3_w = Px_3_Send*sin(bodyYaw) + Py_3_Send*cos(bodyYaw) + bodyYpos
                    # Pz_3_w = Pz_3_Send + 140#bodyZpos+15

                    Px_1_w = Px_1_Send*cos(bodyYaw) - Py_1_Send*sin(bodyYaw) + bodyXpos
                    Py_1_w = Px_1_Send*sin(bodyYaw) + Py_1_Send*cos(bodyYaw) + bodyYpos
                    # Pz_1_w = Pz_1_Send + 140#bodyZpos+15

                    file.write(str(ttime) + "," + str(Px_1_w) + "," + str(Py_1_w) + "," + str(FeetPlaceXBuffer[0]) + "," + str(FeetPlaceYBuffer[0]) + "," + str(Px_3_w) + "," + str(Py_3_w) + "," + str(FeetPlaceXBuffer[2]) + "," + str(FeetPlaceYBuffer[2]) + "," + str(Px_5_w) + "," + str(Py_5_w) + "," + str(FeetPlaceXBuffer[4]) + "," + str(FeetPlaceYBuffer[4])  + "\n")

                # rospy.sleep(0.01)
    except KeyboardInterrupt:
        # file.close()
        print('done')
    except:
        print('aahh noooo')


    


