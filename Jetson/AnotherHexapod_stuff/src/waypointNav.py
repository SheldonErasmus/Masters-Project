#!/usr/bin/env python3

import rospy
from hexapodC import HexapodC
from sensor_msgs.msg import NavSatFix, Imu
from wgs2ned import WGS2NED
from math import atan2, sqrt, cos, sin, pi, copysign
from numpy import array
from ToEulerAngles import ToEulerAng
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point
import time
from GenerateMotionProfile import StepProfile

#Global variables
startupFlag = 1
refLat = 0.0
refLon = 0.0
refAlt = 0.0
n = 0.0
e = 0.0
d = 0.0
curr_Head_hex = 0.0; hex_roll = 0.0; hex_pitch = 0.0


def nav_cb(msg):
    global startupFlag, refLat, refLon, refAlt, n, e, d
    if startupFlag:
        startupFlag = 0
        refLat = msg.latitude
        refLon = msg.longitude
        refAlt = msg.altitude
    Lat = msg.latitude
    Lon = msg.longitude
    Alt = msg.altitude
    (n,e,d) = WGS2NED(refLat,refLon,refAlt,Lat,Lon,Alt)

def imu_cb(msg):
    global curr_Head_hex,hex_roll,hex_pitch
    (hex_roll,hex_pitch,curr_Head_hex) = ToEulerAng(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)
    curr_Head_hex = -curr_Head_hex

if __name__ == '__main__':
    rospy.init_node("WayP_Nav")

    robot = HexapodC()
    stepP = StepProfile(0.1)
    
    subGPS = rospy.Subscriber(robot.ns + "fix", NavSatFix, nav_cb, queue_size=1)
    subIMU = rospy.Subscriber(robot.ns + "imu", Imu, imu_cb, queue_size=1)

    Esrc = float(input("Enter Esrc: "))
    Nsrc = float(input("Enter Nsrc: ") )
    Edest = float(input("Enter Edest: "))
    Ndest = float(input("Enter Ndest: ")) 
      
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    spawn_model_client(model_name='can1',model_xml=open('/home/sheldon/.gazebo/models/cricket_ball/model.sdf', 'r').read(),robot_namespace='/can1',initial_pose=Pose(position=Point(Nsrc,-Esrc,0)),reference_frame='world')
    spawn_model_client(model_name='can2',model_xml=open('/home/sheldon/.gazebo/models/cricket_ball/model.sdf', 'r').read(),robot_namespace='/can2',initial_pose=Pose(position=Point(Ndest,-Edest,0)),reference_frame='world')

    cannum = 2
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

        robot.set_path_var(p=hex_pitch,r=hex_roll)

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

                cannum = cannum +1
                spawn_model_client(model_name='can'+str(cannum) ,model_xml=open('/home/sheldon/.gazebo/models/cricket_ball/model.sdf', 'r').read(),robot_namespace='/can'+str(cannum),initial_pose=Pose(position=Point(Ndest,-Edest,0)),reference_frame='world')
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


