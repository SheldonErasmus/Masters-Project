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
from my_message.msg import PathVar_n_cmdVel
from marvelmind_nav.msg import hedge_pos_ang,marvelmind_waypoint,hedge_imu_fusion, hedge_imu_raw

#Global variables
startupFlag = 1
refLat = 0.0
refLon = 0.0
refAlt = 0.0
n_prev = 0.0
e_prev = 0.0
d_prev = 0.0
curr_Head_hex = 0.0; hex_roll = 0.0; hex_pitch = 0.0

n_cur=0; e_cur=0; d_cur=0; Yaw_meas_pair=0; Flag=0; counter = 98
index=0; total_items=0; movement_type=0; Epoints=0; Npoints=0; DoWaypointFlag=0

def hedge_pos_ang_callback(msg):
    global n_prev, e_prev, d_prev, n_cur, e_cur, d_cur, Yaw_meas_pair, Flag, counter

    counter = counter + 1
    if counter == 100:
        counter = 0
        e_prev = e_cur
        n_prev = n_cur

    e_cur = msg.x_m*1000
    n_cur = msg.y_m*1000
    d_cur = -msg.z_m*1000

    if msg.angle>180:
        Yaw_meas_pair = msg.angle-360
    else:
        Yaw_meas_pair = msg.angle


    Flag = msg.flags
    #print(n_cur, e_cur, d_cur)

Cor_yaw_cur = 0
def imu_cb(msg):
    #global curr_Head_hex,hex_roll,hex_pitch
    #(hex_roll,hex_pitch,curr_Head_hex) = ToEulerAng(msg.qx,msg.qy,msg.qz,msg.qw)
    #curr_Head_hex = -curr_Head_hex
    #print(hex_roll,hex_pitch,curr_Head_hex)

    global Est_yaw_next,Cor_yaw_cur

    if msg.gyro_z>-80 and msg.gyro_z<0:
        Gyro_z = 0
    else:
        Gyro_z = msg.gyro_z*0.0175

    mc1 = 0.5
    mc2 = 0.07
    Ts = 1/100

    Est_yaw_cur = Est_yaw_next

    Cor_yaw_cur = Est_yaw_cur

    n_difference = n_cur-n_prev
    e_difference = e_cur-e_prev
    if abs(n_difference) > 200 or abs(e_difference) > 200: 
        Yaw_meas = atan2(n_difference,e_difference)*180/pi
        Cor_yaw_cur = Cor_yaw_cur + mc1*(Yaw_meas - Est_yaw_cur)
    if 1:
        Cor_yaw_cur = Cor_yaw_cur + mc2*(Yaw_meas_pair - Est_yaw_cur)
    
    Est_yaw_next = Cor_yaw_cur + Ts*Gyro_z
    print(Cor_yaw_cur)

def marvelmind_waypoint_callback(msg):
    global total_items, movement_type, Epoints, Npoints, DoWaypointFlag

    index = msg.item_index
    total_items = msg.total_items
    movement_type = msg.movement_type

    if index == 0:
        Epoints = [None for i in range(total_items)]
        Npoints = [None for i in range(total_items)]

    Epoints[index] = msg.param1*10
    Npoints[index] = msg.param2*10
    #Dpoints[index] = msg.param3
    print(index, total_items, movement_type, Epoints, Npoints)

    if index == total_items-1:
        DoWaypointFlag = 1

if __name__ == '__main__':
    rospy.init_node("WayP_Nav")

    robot = HexapodC()
    stepP = StepProfile(0.1,150)
    
    subIndoorGPS = rospy.Subscriber("hedge_pos_ang", hedge_pos_ang, hedge_pos_ang_callback, queue_size=1)
    rospy.wait_for_message("hedge_pos_ang", hedge_pos_ang)
    Est_yaw_next = Yaw_meas_pair
    subIMU = rospy.Subscriber("hedge_imu_raw", hedge_imu_raw, imu_cb, queue_size=1)
    rospy.Subscriber("marvelmind_waypoint", marvelmind_waypoint, marvelmind_waypoint_callback, queue_size=1) 
    pub = rospy.Publisher('/simple_hexapod/changed_vel_path_var', PathVar_n_cmdVel,queue_size=1)

    PathVelmsg = PathVar_n_cmdVel()
    PathVelmsg.Name = "Waypoint"

    while not rospy.is_shutdown():

        if DoWaypointFlag == 1:

            if index == 0:
                Esrc = Epoints[index]
                Nsrc = Npoints[index]
                Edest = Epoints[index+1]
                Ndest = Npoints[index+1]

                flag = 1

                Head_t = atan2((Edest-Esrc),(Ndest-Nsrc))
                L_t = sqrt((Ndest-Nsrc)**2+(Edest-Esrc)**2)

                TrapezstartTime = time.time()

            if flag == 1:
                ct = array([[cos(Head_t), sin(Head_t)],[-sin(Head_t), cos(Head_t)]]) @ array([[n_cur-Nsrc],[e_cur-Esrc]])
                ct_dist = ct[0]
                ct_err = ct[1]

            if ct_dist < L_t:
                        
                ct_err_ref = 0.0
                err_ref = ct_err_ref - ct_err
                Ky = 0.7
                Head_ref = Head_t + Ky*err_ref
                Head_command = -(Head_ref - (90-Cor_yaw_cur)*pi/180)

                stepP.GenerateProfile(ct_dist,L_t,(time.time()-TrapezstartTime))
                xdot = stepP.F(time.time()-TrapezstartTime)
                V_hexTot = xdot

                if abs(Head_command*180/pi) >= 15:
                    PathVelmsg.linear.x=V_hexTot; PathVelmsg.angular.z=copysign(15,Head_command)
                    pub.publish(PathVelmsg)
                else:
                    PathVelmsg.linear.x=V_hexTot; PathVelmsg.angular.z=Head_command*180/pi
                    pub.publish(PathVelmsg)
                
            else:
                if flag == 1:
                    PathVelmsg.linear.x=0; PathVelmsg.angular.z=0
                    pub.publish(PathVelmsg)

                    if index == total_items-2:
                        index = 0
                        DoWaypointFlag = 0
                        continue
                    else:
                        index = index + 1

                    Esrc = Edest
                    Nsrc = Ndest
                    Edest = Epoints[index+1]
                    Ndest = Npoints[index+1]
                    Head_t = atan2((Edest-Esrc),(Ndest-Nsrc))

                    flag = 0

                if abs(Head_t - (90-Cor_yaw_cur)*pi/180) > 0.05:
                    if abs((Head_t - (90-Cor_yaw_cur)*pi/180)*180/pi) >= 15:
                        PathVelmsg.linear.x=0; PathVelmsg.angular.z=copysign(15,-(Head_t - (90-Cor_yaw_cur)*pi/180))
                        pub.publish(PathVelmsg)
                    else:
                        PathVelmsg.linear.x=0; PathVelmsg.angular.z=-(Head_t - (90-Cor_yaw_cur)*pi/180)*180/pi
                        pub.publish(PathVelmsg)
                else:
                    PathVelmsg.linear.x=0; PathVelmsg.angular.z=0
                    pub.publish(PathVelmsg)
                    L_t = sqrt((Ndest-Nsrc)**2+(Edest-Esrc)**2)
                    flag = 1

                    TrapezstartTime = time.time()

        rospy.sleep(0.1) 


