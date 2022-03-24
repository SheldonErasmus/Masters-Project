#!/usr/bin/env python3

import rospy
from hexapodC import HexapodC
from sensor_msgs.msg import NavSatFix, Imu
from math import atan2, sqrt, cos, sin, pi, copysign
from numpy import array
from ToEulerAngles import ToEulerAng
from geometry_msgs.msg import Pose, Point
import time
from GenerateMotionProfile import StepProfile
from marvelmind_nav.msg import hedge_pos_ang,marvelmind_waypoint,hedge_imu_fusion

#Global variables
startupFlag = 1
refLat = 0.0
refLon = 0.0
refAlt = 0.0
n = 0.0
e = 0.0
d = 0.0
curr_Head_hex = 0.0; hex_roll = 0.0; hex_pitch = 0.0

addr=0; ts=0; Xpos=0; Ypos=0; Zpos=0; Ang=0; Flag=0
index=0; total_items=0; movement_type=0; param1=0; param2=0; param3=0

def hedge_pos_ang_callback(msg):
    global addr, ts, Xpos, Ypos, Zpos, Ang, Flag, e, n

    addr = msg.address
    ts = msg.timestamp_ms
    Xpos = msg.x_m
    e = Xpos
    Ypos = msg.y_m
    n = Ypos
    Zpos = msg.z_m
    Ang = msg.angle
    Flag = msg.flags
    #print(addr, ts, Xpos, Ypos, Zpos, Ang, Flag)

def imu_cb(msg):
    global curr_Head_hex,hex_roll,hex_pitch
    (hex_roll,hex_pitch,curr_Head_hex) = ToEulerAng(msg.qx,msg.qy,msg.qz,msg.qw)
    curr_Head_hex = -curr_Head_hex
    print(hex_roll,hex_pitch,curr_Head_hex)

def marvelmind_waypoint_callback(msg):
    global index, total_items, movement_type, param1, param2, param3

    index = msg.item_index
    total_items = msg.total_items
    movement_type = msg.movement_type
    param1 = msg.param1
    param2 = msg.param2
    param3 = msg.param3
    print(index, total_items, movement_type, param1, param2, param3)

if __name__ == '__main__':
    rospy.init_node("WayP_Nav")

    robot = HexapodC()
    stepP = StepProfile(0.1)
    
    subIndoorGPS = rospy.Subscriber("hedge_pos_ang", hedge_pos_ang, hedge_pos_ang_callback, queue_size=1)
    subIMU = rospy.Subscriber("hedge_imu_fusion", hedge_imu_fusion, imu_cb, queue_size=1)
    rospy.Subscriber("marvelmind_waypoint", marvelmind_waypoint, marvelmind_waypoint_callback, queue_size=1) 

    Esrc = float(input("Enter Esrc: "))
    Nsrc = float(input("Enter Nsrc: ") )
    Edest = float(input("Enter Edest: "))
    Ndest = float(input("Enter Ndest: ")) 

    flag = 1
    prevTime = time.time()

    Head_t = atan2((Edest-Esrc),(Ndest-Nsrc))
    L_t = sqrt((Ndest-Nsrc)**2+(Edest-Esrc)**2)

    ct = array([[cos(Head_t), sin(Head_t)],[-sin(Head_t), cos(Head_t)]]) @ array([[n-Nsrc],[e-Esrc]])
    startPos = ct[0]

    stepP.GenerateProfile(startPos,L_t,150)
    TrapezstartTime = time.time()

    while not rospy.is_shutdown():

        currTime = time.time()
        stepTime = currTime - prevTime
        prevTime = currTime

        #robot.set_path_var(p=hex_pitch,r=hex_roll)

        if flag == 1:
            ct = array([[cos(Head_t), sin(Head_t)],[-sin(Head_t), cos(Head_t)]]) @ array([[n-Nsrc],[e-Esrc]])
            ct_dist = ct[0]
            ct_err = ct[1]

        if ct_dist < L_t:
                    
            ct_err_ref = 0.0
            err_ref = ct_err_ref - ct_err
            Ky = 0.7
            Head_ref = Head_t + Ky*err_ref
            Head_command = -(Head_ref - curr_Head_hex)

            stepP.GenerateProfile(ct_dist,L_t,150-(time.time()-TrapezstartTime))
            xdot = stepP.F(time.time()-TrapezstartTime)
            V_hexTot = xdot

            if abs(Head_command*180/pi) >= 15:
                robot.set_walk_velocity(V_hexTot,0,copysign(15,Head_command))
            else:
                robot.set_walk_velocity(V_hexTot,0,Head_command*180/pi)
            
        else:
            if flag == 1:
                robot.set_walk_velocity(0.0,0,0)
                Esrc = Edest
                Nsrc = Ndest
                Edest = float(input("Enter new Edest: "))
                Ndest = float(input("Enter new Ndest: ") )
                Head_t = atan2((Edest-Esrc),(Ndest-Nsrc))

                flag = 0

            if abs(Head_t - curr_Head_hex) > 0.01:
                if abs((Head_t - curr_Head_hex)*180/pi) >= 15:
                    robot.set_walk_velocity(0.0,0,copysign(15,-(Head_t - curr_Head_hex)))
                else:
                    robot.set_walk_velocity(0.0,0,-(Head_t - curr_Head_hex)*180/pi)
            else:
                robot.set_walk_velocity(0.0,0,0)
                L_t = sqrt((Ndest-Nsrc)**2+(Edest-Esrc)**2)
                flag = 1

                ct = array([[cos(Head_t), sin(Head_t)],[-sin(Head_t), cos(Head_t)]]) @ array([[n-Nsrc],[e-Esrc]])
                startPos = ct[0]

                stepP.GenerateProfile(startPos,L_t,150)
                TrapezstartTime = time.time()

        rospy.sleep(0.1) 


